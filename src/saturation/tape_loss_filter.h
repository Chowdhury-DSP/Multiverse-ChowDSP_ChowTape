#pragma once

#include <chowdsp_filters/chowdsp_filters.h>

namespace ne_pedal::plugins::tape_saturation
{
class TapeLossFilter
{
public:
    TapeLossFilter();

    void setSpeed (float newSpeed) { speed = std::max (newSpeed, 0.5f); }

    void prepare (double sampleRate, int blockSize);

    template <typename Arena>
    void processBlock (const chowdsp::BufferView<float>& buffer, chowdsp::ArenaAllocator<Arena>& allocator) noexcept;

private:
    using HeadBumpFilter = chowdsp::PeakingFilter<float>;

    void calcCoefs (HeadBumpFilter& filter);
    static void calcHeadBumpFilter (float speedIps, float gapMeters, float fs, HeadBumpFilter& filter);

    chowdsp::FIRFilter<float> filters[2];
    HeadBumpFilter bumpFilter[2];
    int activeFilter = 0;
    int fadeCount = 0;
    int fadeLength = 1024;
    chowdsp::BufferView<float> fadeBuffer {};

    float speed = 0.5f;
    float prevSpeed = 0.5f;

    float fs = 44100.0f;
    float fsFactor = 1.0f;
    float binWidth = fs / 100.0f;

    int curOrder = 0;
    std::vector<float> currentCoefs;
    std::vector<float> Hcoefs;
};
}
