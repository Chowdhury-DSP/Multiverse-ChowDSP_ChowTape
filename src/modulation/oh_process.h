#pragma once

#include <chowdsp_dsp_utils/chowdsp_dsp_utils.h>
#include <math_approx/math_approx.hpp>

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

    void prepare (double sampleRate, int samplesPerBlock)
    {
        juce::dsp::ProcessSpec spec { sampleRate, (uint32_t) samplesPerBlock, 1 };

        lpf.prepare (spec);
        lpf.setCutoffFrequency (10.0f);

        sqrtdelta = 1.0f / std::sqrt ((float) sampleRate);
        T = 1.0f / (float) sampleRate;

        y = 1.0f;
    }

    template <typename Arena>
    void prepareBlock (float amtParam, int numSamples, chowdsp::ArenaAllocator<Arena>& allocator)
    {
        read_span = {
            allocator.template allocate<float> (numSamples, chowdsp::SIMDUtils::defaultSIMDAlignment),
            static_cast<size_t> (numSamples),
        };

        for (size_t n = 0; n < read_span.size(); n += 2)
        {
            // Box-Muller transform
            const auto U1 = rand();
            const auto U2 = rand();
            const auto log_U1 = std::min (math_approx::log1p<4> (-U1), 0.0f);
            const auto R = std::sqrt (-(log_U1 + log_U1));
            const auto theta = juce::MathConstants<float>::twoPi * U2 - juce::MathConstants<float>::pi;
            const auto sin_theta = math_approx::sin_mpi_pi<5> (theta);
            const auto cos_theta = math_approx::cos_mpi_pi<5> (theta);
            read_span[n] = R * cos_theta;
            read_span[n + 1] = R * sin_theta;
        }

        amtParam = std::pow (amtParam, 1.25f);
        amt = amtParam;
        damping = amtParam * 20.0f + 1.0f;
        mean = amtParam;
    }

    inline float process (int n) noexcept
    {
        y += sqrtdelta * read_span[n] * amt;
        y += damping * (mean - y) * T;
        return lpf.processSample (0, y);
    }

private:
    float sqrtdelta = 1.0f / std::sqrt (48000.0f);
    float T = 1.0f / 48000.0f;
    float y = 1.0f;

    float amt = 0.0f;
    float mean = 0.0f;
    float damping = 0.0f;

    chowdsp::RandomFloat<float> rand { 123456 };
    nonstd::span<float> read_span;

    chowdsp::NthOrderFilter<float, 2> lpf;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (OHProcess)
};

} // namespace ne_pedal::plugins::tape_mod
