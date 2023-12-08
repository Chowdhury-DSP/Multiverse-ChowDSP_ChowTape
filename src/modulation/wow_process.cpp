#include "wow_process.h"

namespace ne_pedal::plugins::tape_mod
{
void WowProcess::prepare (double sampleRate, int samplesPerBlock, int numChannels)
{
    fs = (float) sampleRate;

    depthSlew.resize ((size_t) numChannels);
    for (auto& dSlew : depthSlew)
    {
        dSlew.reset (sampleRate, 0.05);
        dSlew.setCurrentAndTargetValue (depthSlewMin);
    }

    phase.resize ((size_t) numChannels, 0.0f);

    amp = 1000.0f * 1000.0f / (float) sampleRate;
    wowBuffer.setMaxSize (numChannels, samplesPerBlock);

    ohProc.prepare (sampleRate, samplesPerBlock, numChannels);
}

void WowProcess::prepareBlock (float curDepth, float wowFreq, float wowVar, float wowDrift, int numSamples, int numChannels)
{
    for (auto& dSlew : depthSlew)
        dSlew.setTargetValue (std::max (depthSlewMin, curDepth));

    auto freqAdjust = wowFreq * (1.0f + std::pow ((float) driftRand(), 1.25f) * wowDrift);
    angleDelta = juce::MathConstants<float>::twoPi * freqAdjust / fs;

    wowBuffer.setCurrentSize (numChannels, numSamples);
    wowBuffer.clear();
    wowPtrs = wowBuffer.getArrayOfWritePointers();

    ohProc.prepareBlock (wowVar, numSamples);
}
}
