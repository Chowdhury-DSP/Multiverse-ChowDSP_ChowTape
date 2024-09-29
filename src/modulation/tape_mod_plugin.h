#pragma once

#include <chowdsp_dsp_utils/chowdsp_dsp_utils.h>
#include <float16_t/float16_t.hpp>
#include <plugin_base/PluginBase.h>

#include "flutter_process.h"
#include "wow_process.h"

namespace ne_pedal::plugins::tape_mod
{
class TapeModPlugin : public PluginBase<TapeModPlugin, 6>
{
public:
    explicit TapeModPlugin (const clap_host* host = nullptr);

    static clap_plugin_descriptor description;

    void prepare (double sampleRate, uint32_t samplesPerBlock) override;
    void processBlock (const chowdsp::BufferView<float>& buffer) override;

private:
    void processWetBuffer (const chowdsp::BufferView<float>& buffer);

    FloatParameter flutter_rate_param;
    FloatParameter flutter_depth_param;
    FloatParameter wow_rate_param;
    FloatParameter wow_depth_param;
    FloatParameter wow_var_param;
    FloatParameter wow_drift_param;

    chowdsp::ArenaAllocator<> allocator;

    float fs = 48000.0f;

    WowProcess wow_process;
    FlutterProcess flutter_process;

    static constexpr int HISTORY_SIZE = 1 << 12;
    chowdsp::DelayLine<float,
                       chowdsp::DelayLineInterpolationTypes::Lagrange3rd,
                       numeric::float16_t>
        delay { HISTORY_SIZE };
    chowdsp::SVFHighpass<> dcBlocker;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (TapeModPlugin)
};
} // namespace ne_pedal::plugins::tape_mod
