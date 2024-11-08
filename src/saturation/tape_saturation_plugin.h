#pragma once

#include <chowdsp_dsp_utils/chowdsp_dsp_utils.h>
#include <plugin_base/PluginBase.h>

#include "hysteresis/HysteresisProcessing.h"
#include "tape_loss_filter.h"

namespace ne_pedal::plugins::tape_saturation
{
class TapeSaturationPlugin : public PluginBase<TapeSaturationPlugin, 6>
{
public:
    explicit TapeSaturationPlugin (const clap_host* host = nullptr);

    static clap_plugin_descriptor description;

    void prepare (double sampleRate, uint32_t samplesPerBlock) override;
    void processBlock (const chowdsp::BufferView<float>& buffer) override;

private:
    FloatParameter drive_param;
    FloatParameter saturation_param;
    FloatParameter bias_param;
    FloatParameter tone_param;
    FloatParameter speed_param;
    FloatParameter level_param;

    chowdsp::ArenaAllocator<> allocator;

    chowdsp::SmoothedBufferValue<float> drive_smooth;
    chowdsp::SmoothedBufferValue<float> sat_smooth;
    chowdsp::SmoothedBufferValue<float> bias_smooth;
    chowdsp::SmoothedBufferValue<float> tone_in_smooth;
    chowdsp::SmoothedBufferValue<float> tone_out_smooth;

    float fs = 48000.0f;

    using AAFilter = chowdsp::EllipticFilter<8>;
    chowdsp::Upsampler<float, AAFilter, false> upsampler;
    chowdsp::Downsampler<float, AAFilter, false> downsampler;

    hysteresis::HysteresisProcessing hysteresis_processor;

    chowdsp::ShelfFilter<> tone_in_filter;
    chowdsp::ShelfFilter<> tone_out_filter;

    chowdsp::FirstOrderHPF<float> dcBlocker;
    chowdsp::SmoothedBufferValue<float> gain;

    TapeLossFilter loss_filter;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (TapeSaturationPlugin)
};
}
