#include "tape_mod_plugin.h"

namespace ne_pedal::plugins::tape_mod
{
TapeModPlugin::TapeModPlugin (const clap_host* host)
    : PluginBase (host),
      flutter_rate_param (0, "Flutter Rate", NormalisableRange { 0.0f, 1.0f }, 0.5f),
      flutter_depth_param (1, "Flutter Depth", NormalisableRange { 0.0f, 1.0f }, 0.5f),
      wow_rate_param (2, "Wow Rate", NormalisableRange { 0.0f, 1.0f }, 0.5f),
      wow_depth_param (3, "Wow Depth", NormalisableRange { 0.0f, 1.0f }, 0.5f),
      wow_var_param (4, "Wow Variance", NormalisableRange { 0.0f, 1.0f }, 0.5f),
      wow_drift_param (5, "Wow Drift", NormalisableRange { 0.0f, 1.0f }, 0.5f)
{
    addParameters ({
        &flutter_rate_param,
        &flutter_depth_param,
        &wow_rate_param,
        &wow_depth_param,
        &wow_var_param,
        &wow_drift_param,
    });
}

MAKE_CLAP_DESCRIPTION (TapeModPlugin, "org.chowdsp.tape_mod", "NE-Pedal Tape Mod", "Physically modelled tape modulation effect.")

void TapeModPlugin::prepare (double sampleRate, uint32_t samplesPerBlock)
{
    fs = (float) sampleRate;

    const auto numChannels = 2;
    const auto&& spec = juce::dsp::ProcessSpec { sampleRate, (uint32_t) samplesPerBlock, (uint32_t) numChannels };

    wow_process.prepare (sampleRate, (int) samplesPerBlock, numChannels);
    flutter_process.prepare (sampleRate, (int) samplesPerBlock, numChannels);

    delay.prepare (spec);
    delay.setDelay (0.0f);

    dcBlocker.prepare (spec);
    dcBlocker.setCutoffFrequency (15.0f);
}

void TapeModPlugin::processBlock (const chowdsp::BufferView<float>& buffer)
{
    const auto numChannels = buffer.getNumChannels();
    const auto numSamples = buffer.getNumSamples();

    auto curDepthWow = std::pow (wow_depth_param, 3.0f);
    auto wowFreq = std::pow (4.5f, wow_rate_param) - 1.0f;
    wow_process.prepareBlock (curDepthWow, wowFreq, wow_var_param, wow_drift_param, numSamples, numChannels);

    auto curDepthFlutter = std::pow (std::pow (flutter_depth_param, 3.0f) * 81.0f / 625.0f, 0.5f);
    auto flutterFreq = 0.1f * powf (1000.0f, flutter_rate_param);
    flutter_process.prepareBlock (curDepthFlutter, flutterFreq, numSamples, numChannels);

    processWetBuffer (buffer);

    dcBlocker.processBlock (buffer);
}

void TapeModPlugin::processWetBuffer (const chowdsp::BufferView<float>& buffer)
{
    const auto numSamples = buffer.getNumSamples();
    for (auto [ch, x] : chowdsp::buffer_iters::channels (buffer))
    {
        for (int n = 0; n < numSamples; ++n)
        {
            auto [wowLFO, wowOffset] = wow_process.getLFO (n, ch);
            auto [flutterLFO, flutterOffset] = flutter_process.getLFO (n, ch);

            auto newLength = (wowLFO + flutterLFO + flutterOffset + wowOffset) * fs / 1000.0f;
            newLength = std::clamp (newLength, 0.0f, float (HISTORY_SIZE - 1));

            delay.setDelay (newLength);
            delay.pushSample (ch, x[n]);
            x[n] = delay.popSample (ch);
        }

        wow_process.boundPhase (ch);
        flutter_process.boundPhase (ch);
    }
}
} // namespace ne_pedal::plugins::tape_mod

EXPORT_CLAP_PLUGIN_SYMBOLS (tape_mod::TapeModPlugin)
