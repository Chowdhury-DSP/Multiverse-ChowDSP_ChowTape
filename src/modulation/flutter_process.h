#pragma once

#include <chowdsp_dsp_utils/chowdsp_dsp_utils.h>

namespace ne_pedal::plugins::tape_mod
{
class FlutterProcess
{
public:
    FlutterProcess() = default;

    void prepare (double sampleRate, int samplesPerBlock, int numChannels);
    void prepareBlock (float curDepth, float flutterFreq, int numSamples, int numChannels);

    inline bool shouldTurnOff() const noexcept { return depthSlew[0].getTargetValue() == depthSlewMin; }
    inline void updatePhase (size_t ch) noexcept
    {
        phase1[ch] += angleDelta1;
        phase2[ch] += angleDelta2;
        phase3[ch] += angleDelta3;
    }

    inline std::pair<float, float> getLFO (int n, size_t ch) noexcept
    {
        updatePhase (ch);
        flutterPtrs[ch][n] = depthSlew[ch].getNextValue()
                             * (amp1 * std::cos (phase1[ch] + phaseOff1)
                                + amp2 * std::cos (phase2[ch] + phaseOff2)
                                + amp3 * std::cos (phase3[ch] + phaseOff3));
        return std::make_pair (flutterPtrs[ch][n], dcOffset);
    }

    inline void boundPhase (size_t ch) noexcept
    {
        while (phase1[ch] >= juce::MathConstants<float>::twoPi)
            phase1[ch] -= juce::MathConstants<float>::twoPi;
        while (phase2[ch] >= juce::MathConstants<float>::twoPi)
            phase2[ch] -= juce::MathConstants<float>::twoPi;
        while (phase2[ch] >= juce::MathConstants<float>::twoPi)
            phase2[ch] -= juce::MathConstants<float>::twoPi;
    }


private:
    std::vector<float> phase1;
    std::vector<float> phase2;
    std::vector<float> phase3;

    float amp1 = 0.0f;
    float amp2 = 0.0f;
    float amp3 = 0.0f;
    std::vector<juce::SmoothedValue<float, juce::ValueSmoothingTypes::Multiplicative>> depthSlew;

    float angleDelta1 = 0.0f;
    float angleDelta2 = 0.0f;
    float angleDelta3 = 0.0f;

    float dcOffset = 0.0f;
    static constexpr float phaseOff1 = 0.0f;
    static constexpr float phaseOff2 = 13.0f * juce::MathConstants<float>::pi / 4.0f;
    static constexpr float phaseOff3 = -juce::MathConstants<float>::pi / 10.0f;

    chowdsp::Buffer<float> flutterBuffer;
    float* const* flutterPtrs = nullptr;
    float fs = 48000.0f;

    static constexpr float depthSlewMin = 0.001f;
};
} // namespace ne_pedal::plugins::tape_mod
