#pragma once

#include "HysteresisOps.h"

namespace ne_pedal::plugins::tape_saturation::hysteresis
{
/*
    Hysteresis processing for a model of an analog tape machine.
    For more information on the DSP happening here, see:
    https://ccrma.stanford.edu/~jatin/420/tape/TapeModel_DAFx.pdf
*/
class HysteresisProcessing
{
public:
    HysteresisProcessing();

    void reset();
    void setSampleRate (double newSR);

    void cook (float drive, float width, float sat);

    /* Process a single sample */
    template <typename Float>
    inline Float process (Float H) noexcept
    {
        auto H_d = deriv (H, H_n1, H_d_n1, (Float) T);
        auto M = NRSolver<8> (H, H_d);

        // check for instability
#if HYSTERESIS_USE_SIMD
        auto illCondition = xsimd::isnan (M) | (M > (Float) upperLim);
        M = xsimd::select (illCondition, Float(), M);
        H_d = xsimd::select (illCondition, Float(), H_d);
#else
        bool illCondition = std::isnan (M) || M > upperLim;
        M = illCondition ? 0.0f : M;
        H_d = illCondition ? 0.0f : H_d;
#endif

        M_n1 = M;
        H_n1 = H;
        H_d_n1 = H_d;

        return M;
    }

    void snapToZero()
    {
        auto snapToZero = [] (auto& x, auto floor)
        {
            if (! (x < -floor || x > floor))
                x = 0;
        };

        snapToZero (H_n1, 1.0e-7f);
        snapToZero (H_d_n1, 1.0e-7f);

        if (H_n1 == 0.0f && H_d_n1 == 0.0f)
            M_n1 = 0.0f;
    }

private:
    // newton-raphson solver
    template <int nIterations, typename Float>
    inline Float NRSolver (Float H, Float H_d) noexcept
    {
        Float M = M_n1;
        const Float last_dMdt = hysteresisFunc (M_n1, H_n1, H_d_n1, hpState);

        Float dMdt;
        Float dMdtPrime;
        Float deltaNR;
        int count = 0;
        do
        {
            dMdt = hysteresisFunc (M, H, H_d, hpState);
            dMdtPrime = hysteresisFuncPrime (H_d, dMdt, hpState);
            deltaNR = (M - M_n1 - (Float) Talpha * (dMdt + last_dMdt)) / (Float (1.0) - (Float) Talpha * dMdtPrime);
            M -= deltaNR;
        } while (deltaNR > (Float) 1.0e-4 && ++count < nIterations);

        return M;
    }

    // parameter values
    float fs = 48000.0f;
    float T = 1.0f / fs;
    float Talpha = T / 1.9f;
    float upperLim = 20.0f;

    // state variables
#if HYSTERESIS_USE_SIMD
    HysteresisOps::VecType M_n1 {};
    HysteresisOps::VecType H_n1 {};
    HysteresisOps::VecType H_d_n1 {};
#else
    float M_n1 = 0.0f;
    float H_n1 = 0.0f;
    float H_d_n1 = 0.0f;
#endif

    HysteresisState hpState;
};
} // namespace ne_pedal::plugins::tape_saturation::hysteresis
