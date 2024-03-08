#include "wow_process.h"

namespace ne_pedal::plugins::tape_mod
{
void WowProcess::prepare (double sampleRate, int samplesPerBlock)
{
    fs = (float) sampleRate;

    depthSlew.reset (sampleRate, 0.05);
    depthSlew.setCurrentAndTargetValue (depthSlewMin);

    phase = 0.0f;

    amp = 1000.0f * 1000.0f / (float) sampleRate;

    ohProc.prepare (sampleRate, samplesPerBlock);
}

template <typename Arena>
void WowProcess::prepareBlock (float curDepth, float wowFreq, float wowVar, float wowDrift, int numSamples, chowdsp::ArenaAllocator<Arena>& allocator)
{
    depthSlew.setTargetValue (std::max (depthSlewMin, curDepth));

    auto freqAdjust = wowFreq * (1.0f + std::pow ((float) driftRand(), 1.25f) * wowDrift);
    angleDelta = juce::MathConstants<float>::twoPi * freqAdjust / fs;

    wowBuffer = {
        allocator.template allocate<float> (numSamples, chowdsp::SIMDUtils::defaultSIMDAlignment),
        static_cast<size_t> (numSamples),
    };
    std::fill (wowBuffer.begin(), wowBuffer.end(), 0.0f);

    ohProc.prepareBlock (wowVar, numSamples, allocator);
}

template void WowProcess::prepareBlock (float, float, float, float, int, chowdsp::ArenaAllocator<>&);
} // namespace ne_pedal::plugins::tape_mod
