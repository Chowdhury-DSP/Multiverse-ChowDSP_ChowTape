#include "tape_loss_filter.h"

namespace ne_pedal::plugins::tape_saturation
{
constexpr int baseFIROrder = 64;
constexpr float spacing = 20.0e-6f;
constexpr float thickness = 35.0e-6f;
constexpr float gap = 2.0e-6f;

TapeLossFilter::TapeLossFilter() = default;

void TapeLossFilter::prepare (double sampleRate, int samplesPerBlock, chowdsp::ArenaAllocatorView allocator)
{
    fs = (float) sampleRate;
    fadeLength = std::max (1024, samplesPerBlock);

    fsFactor = (float) fs / 48000.0f;
    curOrder = int ((float) baseFIROrder * fsFactor);

    const auto&& spec = juce::dsp::ProcessSpec { sampleRate, (uint32_t) samplesPerBlock, 1 };
    bumpFilter[0].prepare (spec);
    bumpFilter[1].prepare (spec);
    auto* currentCoefs = calcCoefs (bumpFilter[activeFilter], allocator);

    for (auto& filter : filters)
    {
        filter.setOrder (curOrder);
        filter.prepare (1);
        filter.reset();
        filter.setCoefficients (currentCoefs);
    }
    allocator.clear();

    prevSpeed = speed;
}

float* TapeLossFilter::calcCoefs (HeadBumpFilter& filter, chowdsp::ArenaAllocatorView allocator)
{
    // Set freq domain multipliers
    auto* H = allocator.allocate<float> (curOrder, 8);
    binWidth = fs / (float) curOrder;
    for (int k = 0; k < curOrder / 2; k++)
    {
        const auto freq = (float) k * binWidth;
        const auto waveNumber = juce::MathConstants<float>::twoPi * std::max (freq, 20.0f) / (speed * 0.0254f);
        const auto thickTimesK = waveNumber * thickness;
        const auto kGapOverTwo = waveNumber * gap * 0.5f;

        H[k] = expf (-waveNumber * spacing); // Spacing loss
        H[k] *= (1.0f - expf (-thickTimesK)) / thickTimesK; // Thickness loss
        H[k] *= sinf (kGapOverTwo) / kGapOverTwo; // Gap loss
        H[curOrder - k - 1] = H[k];
    }

    // Create time domain filter signal
    auto* h = allocator.allocate<float> (curOrder, 8);
    std::fill (h, h + curOrder, 0.0f);
    for (int n = 0; n < curOrder / 2; n++)
    {
        const auto idx = (size_t) curOrder / 2 + (size_t) n;
        for (int k = 0; k < curOrder; k++)
            h[idx] += H[k] * cosf (juce::MathConstants<float>::twoPi * (float) k * (float) n / (float) curOrder);

        h[idx] /= (float) curOrder;
        h[curOrder / 2 - n] = h[idx];
    }

    // compute head bump filters
    calcHeadBumpFilter (speed, gap, fs, filter);

    return h;
}

void TapeLossFilter::calcHeadBumpFilter (float speedIps, float gapMeters, float fs, HeadBumpFilter& filter)
{
    auto bumpFreq = speedIps * 0.0254f / (gapMeters * 500.0f);
    auto gain = std::max (1.5f * (1000.0f - std::abs (bumpFreq - 100.0f)) / 1000.0f, 1.0f);
    filter.calcCoefs (bumpFreq, 2.0f, gain, fs);
}

void TapeLossFilter::processBlock (const chowdsp::BufferView<float>& buffer, chowdsp::ArenaAllocatorView allocator) noexcept
{
    const auto arena_frame = allocator.create_frame();
    const auto numSamples = buffer.getNumSamples();

    if ((speed != prevSpeed) && fadeCount == 0)
    {
        auto _ = allocator.create_frame();
        auto* currentCoefs = calcCoefs (bumpFilter[! activeFilter], allocator);
        filters[! activeFilter].setCoefficients (currentCoefs);

        bumpFilter[! activeFilter].reset();

        fadeCount = fadeLength;
        prevSpeed = speed;
    }

    if (fadeCount > 0)
    {
        fadeBuffer = {
            allocator.allocate<float> (numSamples, chowdsp::SIMDUtils::defaultSIMDAlignment),
            numSamples,
        };
        chowdsp::BufferMath::copyBufferData (buffer, fadeBuffer);
    }
    else
    {
        filters[! activeFilter].processBlockBypassed (buffer);
    }

    // normal processing here...
    {
        filters[activeFilter].processBlock (buffer);
        bumpFilter[activeFilter].processBlock (buffer);
    }

    if (fadeCount > 0)
    {
        filters[! activeFilter].processBlock (fadeBuffer);
        bumpFilter[! activeFilter].processBlock (fadeBuffer);

        // fade between buffers
        auto startGain = (float) fadeCount / (float) fadeLength;
        auto samplesToFade = std::min (fadeCount, numSamples);
        fadeCount -= samplesToFade;
        auto endGain = (float) fadeCount / (float) fadeLength;

        { // apply fade
            const auto increment = (endGain - startGain) / (float) std::max (samplesToFade, numSamples);
            auto gain = startGain;
            for (auto [x, fade_x] : chowdsp::zip (buffer.getWriteSpan (0), fadeBuffer.getReadSpan (0)))
            {
                x = gain * x + (1.0f - gain) * fade_x;
                gain += increment;
            }
        }

        if (fadeCount == 0)
            activeFilter = ! activeFilter;
    }
}
}
