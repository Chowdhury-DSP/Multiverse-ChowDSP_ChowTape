#include "HysteresisProcessing.h"
#include <cmath>

namespace ne_pedal::plugins::tape_saturation::hysteresis
{
HysteresisProcessing::HysteresisProcessing() = default;

void HysteresisProcessing::reset()
{
#if HYSTERESIS_USE_SIMD
    const auto zero = HysteresisOps::VecType();
#else
    const auto zero = 0.0;
#endif

    M_n1 = zero;
    H_n1 = zero;
    H_d_n1 = zero;

    hpState.coth = zero;
    hpState.nearZero = false;
}

void HysteresisProcessing::setSampleRate (double newSR)
{
    fs = (float) newSR;
    T = 1.0f / fs;
    Talpha = T / 1.9f;
}

void HysteresisProcessing::cook (float drive, float width, float sat)
{
    hpState.M_s = 0.5f + 1.5f * (1.0f - sat);
    hpState.a = hpState.M_s / (0.01f + 6.0f * drive);
    hpState.c = std::sqrt (1.0f - width) - 0.01f;
    hpState.k = 0.47875f;
    upperLim = 20.0f;

    hpState.nc = 1.0 - hpState.c;
    hpState.M_s_oa = hpState.M_s / hpState.a;
    hpState.M_s_oa_talpha = hpState.alpha * hpState.M_s_oa;
    hpState.M_s_oa_tc = hpState.c * hpState.M_s_oa;
    hpState.M_s_oa_tc_talpha = hpState.alpha * hpState.M_s_oa_tc;
    hpState.M_s_oaSq_tc_talpha = hpState.M_s_oa_tc_talpha / hpState.a;
    hpState.M_s_oaSq_tc_talphaSq = hpState.alpha * hpState.M_s_oaSq_tc_talpha;
}
} // namespace ne_pedal::plugins::tape_saturation::hysteresis
