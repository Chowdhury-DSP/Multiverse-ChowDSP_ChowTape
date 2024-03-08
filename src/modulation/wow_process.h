#pragma once

#include "oh_process.h"

namespace ne_pedal::plugins::tape_mod
{
class WowProcess
{
public:
    WowProcess() = default;

    void prepare (double sampleRate, int samplesPerBlock);

    template <typename Arena>
    void prepareBlock (float curDepth, float wowFreq, float wowVar, float wowDrift, int numSamples, chowdsp::ArenaAllocator<Arena>& allocator);

    inline bool shouldTurnOff() const noexcept { return depthSlew.getTargetValue() == depthSlewMin; }
    inline void updatePhase() noexcept { phase += angleDelta; }

    inline std::pair<float, float> getLFO (int n) noexcept
    {
        updatePhase();
        auto curDepth = depthSlew.getNextValue() * amp;
        wowBuffer[n] = curDepth * (math_approx::cos<5> (phase) + ohProc.process (n));
        return std::make_pair (wowBuffer[n], curDepth);
    }

    inline void boundPhase() noexcept
    {
        while (phase >= juce::MathConstants<float>::twoPi)
            phase -= juce::MathConstants<float>::twoPi;
    }

private:
    float angleDelta = 0.0f;
    float amp = 0.0f;
    float phase = 0.0f;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Multiplicative> depthSlew;

    nonstd::span<float> wowBuffer;
    float fs = 44100.0f;

    OHProcess ohProc;
    chowdsp::RandomFloat<float> driftRand { 0x45698 };

    static constexpr float depthSlewMin = 0.001f;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (WowProcess)
};
} // namespace ne_pedal::plugins::tape_mod
