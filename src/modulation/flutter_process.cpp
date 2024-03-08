#include "flutter_process.h"

namespace ne_pedal::plugins::tape_mod
{
void FlutterProcess::prepare (double sampleRate, int samplesPerBlock)
{
    fs = static_cast<float> (sampleRate);

    depthSlew.reset (sampleRate, 0.05);
    depthSlew.setCurrentAndTargetValue (depthSlewMin);

    phase1 = 0.0f;
    phase2 = 0.0f;
    phase3 = 0.0f;

    amp1 = -230.0f * 1000.0f / fs;
    amp2 = -80.0f * 1000.0f / fs;
    amp3 = -99.0f * 1000.0f / fs;
    dcOffset = 350.0f * 1000.0f / fs;
}

template <typename Arena>
void FlutterProcess::prepareBlock (float curDepth, float flutterFreq, int numSamples, chowdsp::ArenaAllocator<Arena>& allocator)
{
    depthSlew.setTargetValue (std::max (depthSlewMin, curDepth));

    angleDelta1 = juce::MathConstants<float>::twoPi * flutterFreq / fs;
    angleDelta2 = 2.0f * angleDelta1;
    angleDelta3 = 3.0f * angleDelta1;

    flutterBuffer = {
        allocator.template allocate<float> (numSamples, chowdsp::SIMDUtils::defaultSIMDAlignment),
        static_cast<size_t> (numSamples),
    };
    std::fill (flutterBuffer.begin(), flutterBuffer.end(), 0.0f);
}

template void FlutterProcess::prepareBlock (float, float, int, chowdsp::ArenaAllocator<>&);
} // namespace ne_pedal::plugins::tape_mod
