#pragma once

#include <cmath>
#include <chowdsp_math/chowdsp_math.h>

namespace ne_pedal::plugins::tape_saturation::hysteresis
{
// we're only using hysteresis on mono signals, so there's no point in doing SIMD
#define HYSTERESIS_USE_SIMD 0
//#if JUCE_DEBUG
//#define HYSTERESIS_USE_SIMD 0
//#else
//#define HYSTERESIS_USE_SIMD 1
//#endif

#if HYSTERESIS_USE_SIMD
using VecType = xsimd::batch<double>;
using MaskType = VecType::batch_bool_type;
#else
using VecType = float;
using MaskType = bool;
#endif

struct HysteresisState
{
    // variable names in this struct are all matched with the
    // corresponding variable names in the reference paper

    // parameter values
    double M_s = 1.0; // Magnetic saturation (see eq. 9)
    double a = M_s / 4.0; // shape of the anhysteric magnetisation (eq. 9)
    static constexpr double alpha = 1.6e-3; // mean field parameter (eq. 6)
    double k = 0.47875; // measure for the width of the hysteresis loop (eq. 6)
    double c = 1.7e-1; // ratio of the normal and anhysteric initial susceptibilities (eq. 6)

    // Save calculations
    double nc = 1 - c;
    double M_s_oa = M_s / a;
    double M_s_oa_talpha = alpha * M_s / a;
    double M_s_oa_tc = c * M_s / a;
    double M_s_oa_tc_talpha = alpha * c * M_s / a;
    double M_s_oaSq_tc_talpha = alpha * c * M_s / (a * a);
    double M_s_oaSq_tc_talphaSq = alpha * alpha * c * M_s / (a * a);

    // temp vars
#if HYSTERESIS_USE_SIMD
    VecType Q, M_diff, L_prime, kap1, f1Denom, f1, f2, f3;
    VecType coth = VecType (0.0);
    MaskType nearZero;
#else
    double Q, M_diff, L_prime, kap1, f1Denom, f1, f2, f3;
    double coth = 0.0;
    bool nearZero = false;
    double oneOverQ, oneOverQSq, oneOverQCubed, cothSq, oneOverF3, oneOverF1Denom;
#endif
};

constexpr double ONE_THIRD = 1.0 / 3.0;
constexpr double NEG_TWO_OVER_15 = -2.0 / 15.0;

constexpr inline int sign (double x)
{
    return int (x > 0.0) - int (x < 0.0);
}

/** Langevin function */
template <typename Float>
static inline Float langevin (const HysteresisState& hp) noexcept
{
#if HYSTERESIS_USE_SIMD
    return xsimd::select (nearZero, x / 3.0, coth - ((Float) 1.0 / x));
#else
    return ! hp.nearZero ? (hp.coth) - (hp.oneOverQ) : hp.Q * ONE_THIRD;
#endif
}

/** Derivative of Langevin function */
template <typename Float>
static inline Float langevinD (const HysteresisState& hp) noexcept
{
#if HYSTERESIS_USE_SIMD
    return xsimd::select (nearZero, (Float) ONE_THIRD, ((Float) 1.0 / (x * x)) - (coth * coth) + 1.0);
#else
    return ! hp.nearZero ? hp.oneOverQSq - hp.cothSq + 1.0 : ONE_THIRD;
#endif
}

/** 2nd derivative of Langevin function */
template <typename Float>
static inline Float langevinD2 (const HysteresisState& hp) noexcept
{
#if HYSTERESIS_USE_SIMD
    return xsimd::select (
        nearZero, x * NEG_TWO_OVER_15, (Float) 2.0 * coth * (coth * coth - 1.0) - ((Float) 2.0 / (x * x * x)));
#else
    return ! hp.nearZero ? 2.0 * hp.coth * (hp.cothSq - 1.0) - (2.0 * hp.oneOverQCubed) : NEG_TWO_OVER_15 * hp.Q;
#endif
}

/** Derivative by alpha transform */
template <typename Float>
static inline Float deriv (Float x_n, Float x_n1, Float x_d_n1, Float T) noexcept
{
    const Float dAlpha = VecType (0.75);
    return ((((Float) 1.0 + dAlpha) / T) * (x_n - x_n1)) - dAlpha * x_d_n1;
}

template <typename Float>
static inline Float fast_coth (Float x) noexcept
{
    using namespace chowdsp::Polynomials;

    const auto x_sq = x * x;

    // from: https://mathr.co.uk/blog/2017-09-06_approximating_hyperbolic_tangent.html
    // [7/6]: x*(135135+17325*x**2+378*x**4+x**6)/(135135+62370*x**2+3150*x**4+28*x**6)
    const auto tanh_numerator = x * estrin<3> ({ 1.0, 378.0, 17325.0, 135135.0 }, x_sq);
    const auto tanh_denominator = estrin<3> ({ 28.0, 3150.0, 62370.0, 135135.0 }, x_sq);

    // coth is the reciprocal of tanh
    return tanh_denominator / tanh_numerator;
}

/** hysteresis function dM/dt */
template <typename Float>
static inline Float hysteresisFunc (Float M, Float H, Float H_d, HysteresisState& hp) noexcept
{
    hp.Q = (H + M * hp.alpha) * (1.0 / hp.a);
    hp.oneOverQ = (Float) 1.0 / hp.Q;
    hp.oneOverQSq = hp.oneOverQ * hp.oneOverQ;
    hp.oneOverQCubed = hp.oneOverQ * hp.oneOverQSq;

#if HYSTERESIS_USE_SIMD
    hp.coth = (Float) 1.0 / xsimd::tanh (hp.Q);
    hp.nearZero = (hp.Q < (Float) 0.001) && (hp.Q > (Float) -0.001);
#else
    hp.coth = fast_coth (hp.Q);
    hp.nearZero = hp.Q < 0.001 && hp.Q > -0.001;
#endif

    hp.cothSq = hp.coth * hp.coth;
    hp.M_diff = langevin<Float> (hp) * hp.M_s - M;

#if HYSTERESIS_USE_SIMD
    const auto delta = xsimd::select (H_d >= 0.0, (Float) 1.0, (Float) -1.0);
    const auto delta_M = signumSIMD (delta) == signumSIMD (hp.M_diff);
    hp.kap1 = xsimd::select (delta_M, (Float) hp.nc, (Float) 0.0);
#else
    const auto delta = (Float) ((H_d >= 0.0) - (H_d < 0.0));
    const auto delta_M = (Float) (sign (delta) == sign (hp.M_diff));
    hp.kap1 = (Float) hp.nc * delta_M;
#endif

    hp.L_prime = langevinD<Float> (hp);

    hp.f1Denom = ((Float) hp.nc * delta) * hp.k - (Float) hp.alpha * hp.M_diff;
    hp.oneOverF1Denom = (Float) 1.0 / hp.f1Denom;
    hp.f1 = hp.kap1 * hp.M_diff * hp.oneOverF1Denom;
    hp.f2 = hp.L_prime * hp.M_s_oa_tc;
    hp.f3 = (Float) 1.0 - (hp.L_prime * hp.M_s_oa_tc_talpha);
    hp.oneOverF3 = (Float) 1.0 / hp.f3;

    return H_d * (hp.f1 + hp.f2) * hp.oneOverF3;
}

// derivative of hysteresis func w.r.t M (depends on cached values from computing hysteresisFunc)
template <typename Float>
static inline Float hysteresisFuncPrime (Float H_d, Float dMdt, HysteresisState& hp) noexcept
{
    const Float L_prime2 = langevinD2<Float> (hp);
    const Float M_diff2 = hp.L_prime * hp.M_s_oa_talpha - 1.0;

    const Float f1_p = hp.kap1 * ((M_diff2 * hp.oneOverF1Denom) + hp.M_diff * HysteresisState::alpha * M_diff2 * (hp.oneOverF1Denom * hp.oneOverF1Denom));
    const Float f2_p = L_prime2 * hp.M_s_oaSq_tc_talpha;
    const Float f3_p = L_prime2 * (-hp.M_s_oaSq_tc_talphaSq);

    return (H_d * (f1_p + f2_p) - dMdt * f3_p) * hp.oneOverF3;
}
} // namespace ne_pedal::plugins::tape_saturation::hysteresis
