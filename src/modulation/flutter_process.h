#pragma once

#include <chowdsp_dsp_utils/chowdsp_dsp_utils.h>
#include <math_approx/math_approx.hpp>

namespace ne_pedal::plugins::tape_mod
{
class FlutterProcess
{
public:
    FlutterProcess() = default;

    void prepare (double sampleRate, int samplesPerBlock);

    template <typename Arena>
    void prepareBlock (float curDepth, float flutterFreq, int numSamples, chowdsp::ArenaAllocator<Arena>& allocator);

    inline bool shouldTurnOff() const noexcept { return depthSlew.getTargetValue() == depthSlewMin; }
    inline void updatePhase() noexcept
    {
        phase1 += angleDelta1;
        phase2 += angleDelta2;
        phase3 += angleDelta3;
    }

    inline std::pair<float, float> getLFO (int n) noexcept
    {
        updatePhase();
        flutterBuffer[n] = depthSlew.getNextValue()
                             * (amp1 * math_approx::cos<5> (phase1 + phaseOff1)
                                + amp2 * math_approx::cos<5> (phase2 + phaseOff2)
                                + amp3 * math_approx::cos<5> (phase3 + phaseOff3));
        return std::make_pair (flutterBuffer[n], dcOffset);
    }

    inline void boundPhase (size_t ch) noexcept
    {
        while (phase1 >= juce::MathConstants<float>::twoPi)
            phase1 -= juce::MathConstants<float>::twoPi;
        while (phase2 >= juce::MathConstants<float>::twoPi)
            phase2 -= juce::MathConstants<float>::twoPi;
        while (phase2 >= juce::MathConstants<float>::twoPi)
            phase2 -= juce::MathConstants<float>::twoPi;
    }


private:
    float phase1 {};
    float phase2 {};
    float phase3 {};

    float amp1 = 0.0f;
    float amp2 = 0.0f;
    float amp3 = 0.0f;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Multiplicative> depthSlew;

    float angleDelta1 = 0.0f;
    float angleDelta2 = 0.0f;
    float angleDelta3 = 0.0f;

    float dcOffset = 0.0f;
    static constexpr float phaseOff1 = 0.0f;
    static constexpr float phaseOff2 = 13.0f * juce::MathConstants<float>::pi / 4.0f;
    static constexpr float phaseOff3 = -juce::MathConstants<float>::pi / 10.0f;

    nonstd::span<float> flutterBuffer;
    float* const* flutterPtrs = nullptr;
    float fs = 48000.0f;

    static constexpr float depthSlewMin = 0.001f;
};
} // namespace ne_pedal::plugins::tape_mod
