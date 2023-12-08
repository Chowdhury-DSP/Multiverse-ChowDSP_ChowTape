#pragma once

#include <chowdsp_dsp_utils/chowdsp_dsp_utils.h>

namespace chowdsp
{
// borrowed from: https://audiodev.blog/random-numbers/
struct RandomGenerator
{
    RandomGenerator() = default;
    explicit RandomGenerator (uint64_t seed) : seed (seed) {}

    double operator()()
    {
        seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
        return double ((seed >> 11) & 0x1FFFFFFFFFFFFFULL) / 9007199254740992.0;
    }

private:
    uint64_t seed = 161803398ULL;
};
} // namespace chowdsp

namespace ne_pedal::plugins::tape_mod
{
/**
 * Class to simulate the Ornstein-Uhlenbeck process.
 * Mostly lifted from https://github.com/mhampton/ZetaCarinaeModules
 * under the GPLv3 license.
 */
class OHProcess
{
public:
    OHProcess() = default;

    void prepare (double sampleRate, int samplesPerBlock, int numChannels)
    {
        juce::dsp::ProcessSpec spec { sampleRate, (uint32_t) samplesPerBlock, (uint32_t) numChannels };

        lpf.prepare (spec);
        lpf.setCutoffFrequency (10.0f);

        noiseBuffer.setMaxSize (1, samplesPerBlock);
        rPtr = noiseBuffer.getReadPointer (0);

        sqrtdelta = 1.0f / std::sqrt ((float) sampleRate);
        T = 1.0f / (float) sampleRate;

        y.resize ((size_t) numChannels, 0.0f);
        y[0] = 1.0f;
    }

    void prepareBlock (float amtParam, int numSamples)
    {
        noiseBuffer.setCurrentSize (1, numSamples);
        noiseBuffer.clear();

        for (auto& x : noiseBuffer.getWriteSpan (0))
        {
            // Box-Muller transform
            const auto radius = std::sqrt ((T) -2 * std::log (1.0f - (float) rand()));
            const auto theta = juce::MathConstants<float>::twoPi * (float) rand();
            const auto value = radius * std::sin (theta) / juce::MathConstants<float>::sqrt2;
            x = (1.0f / 2.33f) * value;
        }

        amtParam = std::pow (amtParam, 1.25f);
        amt = amtParam;
        damping = amtParam * 20.0f + 1.0f;
        mean = amtParam;
    }

    inline float process (int n, size_t ch) noexcept
    {
        y[ch] += sqrtdelta * rPtr[n] * amt;
        y[ch] += damping * (mean - y[ch]) * T;
        return lpf.processSample ((int) ch, y[ch]);
    }

private:
    float sqrtdelta = 1.0f / std::sqrt (48000.0f);
    float T = 1.0f / 48000.0f;
    std::vector<float> y;

    float amt = 0.0f;
    float mean = 0.0f;
    float damping = 0.0f;

    chowdsp::RandomGenerator rand { 123456 };
    chowdsp::Buffer<float> noiseBuffer;
    const float* rPtr = nullptr;

    chowdsp::NthOrderFilter<float, 2> lpf;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (OHProcess)
};

} // namespace ne_pedal::plugins::tape_mod
