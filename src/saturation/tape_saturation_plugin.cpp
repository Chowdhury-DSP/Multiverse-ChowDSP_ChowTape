#include "tape_saturation_plugin.h"

namespace ne_pedal::plugins::tape_saturation
{
constexpr int os_ratio = 3;

TapeSaturationPlugin::TapeSaturationPlugin (const clap_host* host)
    : PluginBase (host),
      drive_param (0, "Drive", NormalisableRange { 0.0f, 1.0f }, 0.5f),
      saturation_param (1, "Saturation", NormalisableRange { 0.0f, 1.0f }, 0.5f),
      bias_param (2, "Bias", NormalisableRange { 0.0f, 1.0f }, 0.5f),
      tone_param (3, "Tone", NormalisableRange { -1.0f, 1.0f }, 0.0f),
      speed_param (4, "Speed", NormalisableRange { 0.5f, 50.0f }, 15.0f),
      level_param (5, "Level", NormalisableRange { 0.0f, 1.0f }, 0.75f)
{
    addParameters ({ &drive_param,
                     &saturation_param,
                     &bias_param,
                     &tone_param,
                     &speed_param,
                     &level_param });
}

MAKE_CLAP_DESCRIPTION (TapeSaturationPlugin, "org.chowdsp.plugin", "NE-Pedal Tape Saturation", "Physically modelled tape saturation effect.")

void TapeSaturationPlugin::prepare (double sampleRate, uint32_t samplesPerBlock)
{
    fs = (float) sampleRate;

    const auto ds_spec = juce::dsp::ProcessSpec { sampleRate, samplesPerBlock, 1 };
    const auto os_spec = juce::dsp::ProcessSpec { os_ratio * sampleRate, os_ratio * samplesPerBlock, 1 };

    tone_in_filter.prepare (ds_spec);
    tone_out_filter.prepare (ds_spec);

    tone_in_smooth.setRampLength (0.05);
    tone_in_smooth.prepare (sampleRate, (int) samplesPerBlock);

    tone_out_smooth.setRampLength (0.05);
    tone_out_smooth.prepare (sampleRate, (int) samplesPerBlock);

    upsampler.prepare (ds_spec, os_ratio);
    downsampler.prepare (os_spec, os_ratio);

    hysteresis_processor.reset();
    hysteresis_processor.setSampleRate (os_spec.sampleRate);

    drive_smooth.setRampLength (0.05);
    drive_smooth.prepare (os_spec.sampleRate, (int) os_spec.maximumBlockSize);
    sat_smooth.setRampLength (0.05);
    sat_smooth.prepare (os_spec.sampleRate, (int) os_spec.maximumBlockSize);
    bias_smooth.setRampLength (0.05);
    bias_smooth.prepare (os_spec.sampleRate, (int) os_spec.maximumBlockSize);

    dcBlocker.calcCoefs (16.0f, (float) sampleRate);
    dcBlocker.reset();

    loss_filter.setSpeed (speed_param);
    loss_filter.prepare (sampleRate, (int) samplesPerBlock);

    gain_processor.setRampDurationSeconds (0.05);
    gain_processor.prepare (ds_spec);
}

void TapeSaturationPlugin::processBlock (const chowdsp::BufferView<float>& buffer)
{
    const auto update_tone_filter = [this] (chowdsp::ShelfFilter<float>& filter, float current_tone_param, bool is_input)
    {
        static constexpr auto tone_filter_transition_freq = 750.0f;
        if (current_tone_param > 0.0f)
        {
            const auto low_freq_gain = juce::Decibels::decibelsToGain (current_tone_param * 12.0f);
            filter.calcCoefs (is_input ? low_freq_gain : (1.0f / low_freq_gain), 1.0f, tone_filter_transition_freq, fs);
        }
        else
        {
            const auto high_freq_gain = juce::Decibels::decibelsToGain (current_tone_param * -12.0f);
            filter.calcCoefs (1.0f, is_input ? high_freq_gain : (1.0f / high_freq_gain), tone_filter_transition_freq, fs);
        }
    };

    tone_in_smooth.process ((float) tone_param, buffer.getNumSamples());
    if (tone_in_smooth.isSmoothing())
    {
        const auto* tone_data = tone_in_smooth.getSmoothedBuffer();
        for (auto [n, sample] : chowdsp::enumerate (buffer.getWriteSpan (0)))
        {
            update_tone_filter (tone_in_filter, tone_data[n], true);
            sample = tone_in_filter.processSample (sample);
        }
    }
    else
    {
        update_tone_filter (tone_in_filter, tone_in_smooth.getCurrentValue(), true);
        tone_in_filter.processBlock (buffer);
    }

    const auto os_buffer = upsampler.process (buffer);

    for (auto& sample : os_buffer.getWriteSpan (0))
        sample = chowdsp::Math::algebraicSigmoid (sample);

    drive_smooth.process ((float) drive_param, os_buffer.getNumSamples());
    sat_smooth.process ((float) saturation_param, os_buffer.getNumSamples());
    bias_smooth.process ((float) bias_param, os_buffer.getNumSamples());
    if (drive_smooth.isSmoothing() || sat_smooth.isSmoothing() || bias_smooth.isSmoothing())
    {
        const auto* drive_data = drive_smooth.getSmoothedBuffer();
        const auto* sat_data = sat_smooth.getSmoothedBuffer();
        const auto* bias_data = bias_smooth.getSmoothedBuffer();

        for (auto [n, sample] : chowdsp::enumerate (os_buffer.getWriteSpan (0)))
        {
            hysteresis_processor.cook (drive_data[n], 1.0f - bias_data[n], sat_data[n]);
            sample = hysteresis_processor.process (sample);
        }
    }
    else
    {
        hysteresis_processor.cook (drive_smooth.getCurrentValue(),
                                   1.0f - bias_smooth.getCurrentValue(),
                                   sat_smooth.getCurrentValue());
        for (auto& sample : os_buffer.getWriteSpan (0))
            sample = hysteresis_processor.process (sample);
    }
    hysteresis_processor.snapToZero();

    downsampler.process (os_buffer, buffer);

    tone_out_smooth.process ((float) tone_param, buffer.getNumSamples());
    if (tone_out_smooth.isSmoothing())
    {
        const auto* tone_data = tone_out_smooth.getSmoothedBuffer();
        for (auto [n, sample] : chowdsp::enumerate (buffer.getWriteSpan (0)))
        {
            update_tone_filter (tone_out_filter, tone_data[n], false);
            sample = tone_out_filter.processSample (sample);
        }
    }
    else
    {
        update_tone_filter (tone_out_filter, tone_out_smooth.getCurrentValue(), false);
        tone_out_filter.processBlock (buffer);
    }

    dcBlocker.processBlock (buffer);

    loss_filter.setSpeed (speed_param);
    loss_filter.processBlock (buffer);

    const auto gain = juce::Decibels::decibelsToGain (48.0f * (float) level_param - 24.0f, -24.0f);
    gain_processor.setGainLinear (gain);
    gain_processor.process (buffer);

    chowdsp::BufferMath::sanitizeBuffer (buffer, 5.0f);
}
} // namespace ne_pedal::plugins::tape_saturation

EXPORT_CLAP_PLUGIN_SYMBOLS (tape_saturation::TapeSaturationPlugin)
