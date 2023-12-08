#pragma once

#include "oh_process.h"

namespace ne_pedal::plugins::tape_mod
{
class WowProcess
{
public:
    WowProcess() = default;

    void prepare (double sampleRate, int samplesPerBlock, int numChannels);
    void prepareBlock (float curDepth, float wowFreq, float wowVar, float wowDrift, int numSamples, int numChannels);

    inline bool shouldTurnOff() const noexcept { return depthSlew[0].getTargetValue() == depthSlewMin; }
    inline void updatePhase (size_t ch) noexcept { phase[ch] += angleDelta; }

    inline std::pair<float, float> getLFO (int n, size_t ch) noexcept
    {
        updatePhase (ch);
        auto curDepth = depthSlew[ch].getNextValue() * amp;
        wowPtrs[ch][n] = curDepth * (std::cos (phase[ch]) + ohProc.process (n, ch));
        return std::make_pair (wowPtrs[ch][n], curDepth);
    }

    inline void boundPhase (size_t ch) noexcept
    {
        while (phase[ch] >= juce::MathConstants<float>::twoPi)
            phase[ch] -= juce::MathConstants<float>::twoPi;
    }

private:
    float angleDelta = 0.0f;
    float amp = 0.0f;
    std::vector<float> phase;
    std::vector<juce::SmoothedValue<float, juce::ValueSmoothingTypes::Multiplicative>> depthSlew;

    chowdsp::Buffer<float> wowBuffer;
    float* const* wowPtrs = nullptr;
    float fs = 44100.0f;

    OHProcess ohProc;
    chowdsp::RandomGenerator driftRand;

    static constexpr float depthSlewMin = 0.001f;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (WowProcess)
};
} // namespace ne_pedal::plugins::tape_mod
