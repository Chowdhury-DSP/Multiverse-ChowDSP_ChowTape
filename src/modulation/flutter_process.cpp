#include "flutter_process.h"

namespace ne_pedal::plugins::tape_mod
{
void FlutterProcess::prepare (double sampleRate, int samplesPerBlock, int numChannels)
{
    fs = (float) sampleRate;

    depthSlew.resize ((size_t) numChannels);
    for (auto& dSlew : depthSlew)
    {
        dSlew.reset (sampleRate, 0.05);
        dSlew.setCurrentAndTargetValue (depthSlewMin);
    }

    phase1.resize ((size_t) numChannels, 0.0f);
    phase2.resize ((size_t) numChannels, 0.0f);
    phase3.resize ((size_t) numChannels, 0.0f);

    amp1 = -230.0f * 1000.0f / fs;
    amp2 = -80.0f * 1000.0f / fs;
    amp3 = -99.0f * 1000.0f / fs;
    dcOffset = 350.0f * 1000.0f / fs;

    flutterBuffer.setMaxSize (numChannels, samplesPerBlock);

}

void FlutterProcess::prepareBlock (float curDepth, float flutterFreq, int numSamples, int numChannels)
{
    for (auto& dSlew : depthSlew)
        dSlew.setTargetValue (std::max (depthSlewMin, curDepth));

    angleDelta1 = juce::MathConstants<float>::twoPi * flutterFreq / fs;
    angleDelta2 = 2.0f * angleDelta1;
    angleDelta3 = 3.0f * angleDelta1;

    flutterBuffer.setCurrentSize (numChannels, numSamples);
    flutterBuffer.clear();
    flutterPtrs = flutterBuffer.getArrayOfWritePointers();

}
} // namespace ne_pedal::plugins::tape_mod
