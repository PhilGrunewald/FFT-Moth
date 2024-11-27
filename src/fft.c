/****************************************************************************
 * fft.c
 * openacousticdevices.info
 * November 2024
 *****************************************************************************/
 
#include <math.h>
#include <stdint.h>

#include "fft_tables_1024.h"

/* Radix functions */

static inline void singleRealTransform2(int16_t *dataBuffer, uint32_t index, uint32_t step, uint32_t outOffset, float *fftBuffer) {

    const float evenR = (float)dataBuffer[index] * coefficients[index];
    const float oddR = (float)dataBuffer[index + step] * coefficients[index + step];

    const float leftR = evenR + oddR;
    const float rightR = evenR - oddR;

    fftBuffer[outOffset] = leftR;
    fftBuffer[outOffset + 1] = 0;
    fftBuffer[outOffset + 2] = rightR;
    fftBuffer[outOffset + 3] = 0;

}

static inline void singleRealTransform4(int16_t *dataBuffer, uint32_t index, uint32_t step, uint32_t outOffset, float *fftBuffer) {

    const float Ar = (float)dataBuffer[index] * coefficients[index];
    const float Br = (float)dataBuffer[index + step] * coefficients[index + step];
    const float Cr = (float)dataBuffer[index + 2 * step] * coefficients[index + 2 * step];
    const float Dr = (float)dataBuffer[index + 3 * step] * coefficients[index + 3 * step];

    const float T0r = Ar + Cr;
    const float T1r = Ar - Cr;
    const float T2r = Br + Dr;
    const float T3r = Br - Dr;

    const float FAr = T0r + T2r;
    const float FBr = T1r;
    const float FBi = -T3r;
    const float FCr = T0r - T2r;
    const float FDr = T1r;
    const float FDi = T3r;

    fftBuffer[outOffset] = FAr;
    fftBuffer[outOffset + 1] = 0;
    fftBuffer[outOffset + 2] = FBr;
    fftBuffer[outOffset + 3] = FBi;
    fftBuffer[outOffset + 4] = FCr;
    fftBuffer[outOffset + 5] = 0;
    fftBuffer[outOffset + 6] = FDr;
    fftBuffer[outOffset + 7] = FDi;  

}

/* Public functions */

void FFT_realTransform(int16_t *dataBuffer, float *fftBuffer) {

    /* Initialise counters */

    uint32_t step = 1 << WIDTH;

    uint32_t len = (CSIZE / step) << 1;

    /* Call initial transform functions */

    if (len == 4) {

        for (uint32_t outputOffset = 0, t = 0; outputOffset < CSIZE; outputOffset += len, t++) {

            singleRealTransform2(dataBuffer, bitReversalTable[t] >> 1, step >> 1, outputOffset, fftBuffer);

        }

    } else {

        for (uint32_t outputOffset = 0, t = 0; outputOffset < CSIZE; outputOffset += len, t++) {

            singleRealTransform4(dataBuffer, bitReversalTable[t] >> 1, step >> 1, outputOffset, fftBuffer);

        }

    }

    /* Complete transform */

    for (step >>= 2; step >= 2; step >>= 2) {

        len = (CSIZE / step) << 1;

        const uint32_t halfLen = len >> 1;
        const uint32_t quarterLen = halfLen >> 1;
        const uint32_t halfQuarterLen = quarterLen >> 1;

        for (uint32_t outputOffset = 0; outputOffset < CSIZE; outputOffset += len) {

            for (uint32_t i = 0, k = 0; i <= halfQuarterLen; i += 2, k += step) {

                const uint32_t A = outputOffset + i;
                const uint32_t B = A + quarterLen;
                const uint32_t C = B + quarterLen;
                const uint32_t D = C + quarterLen;

                const float Ar = fftBuffer[A];
                const float Ai = fftBuffer[A + 1];
                const float Br = fftBuffer[B];
                const float Bi = fftBuffer[B + 1];
                const float Cr = fftBuffer[C];
                const float Ci = fftBuffer[C + 1];
                const float Dr = fftBuffer[D];
                const float Di = fftBuffer[D + 1];

                const float MAr = Ar;
                const float MAi = Ai;

                const float tableBr = trigonometryTable[k];
                const float tableBi = trigonometryTable[k + 1];
                const float MBr = Br * tableBr - Bi * tableBi;
                const float MBi = Br * tableBi + Bi * tableBr;

                const float tableCr = trigonometryTable[2 * k];
                const float tableCi = trigonometryTable[2 * k + 1];
                const float MCr = Cr * tableCr - Ci * tableCi;
                const float MCi = Cr * tableCi + Ci * tableCr;

                const float tableDr = trigonometryTable[3 * k];
                const float tableDi = trigonometryTable[3 * k + 1];
                const float MDr = Dr * tableDr - Di * tableDi;
                const float MDi = Dr * tableDi + Di * tableDr;

                const float T0r = MAr + MCr;
                const float T0i = MAi + MCi;
                const float T1r = MAr - MCr;
                const float T1i = MAi - MCi;
                const float T2r = MBr + MDr;
                const float T2i = MBi + MDi;
                const float T3r = MBr - MDr;
                const float T3i = MBi - MDi;

                const float FAr = T0r + T2r;
                const float FAi = T0i + T2i;

                const float FBr = T1r + T3i;
                const float FBi = T1i - T3r;

                fftBuffer[A] = FAr;
                fftBuffer[A + 1] = FAi;
                fftBuffer[B] = FBr;
                fftBuffer[B + 1] = FBi;

                if (i == 0) {

                    const float FCr = T0r - T2r;
                    const float FCi = T0i - T2i;

                    fftBuffer[C] = FCr;
                    fftBuffer[C + 1] = FCi;

                    continue;

                }

                if (i == halfQuarterLen) continue;

                const float ST0r = T1r;
                const float ST0i = -T1i;
                const float ST1r = T0r;
                const float ST1i = -T0i;
                const float ST2r = -T3i;
                const float ST2i = -T3r;
                const float ST3r = -T2i;
                const float ST3i = -T2r;

                const float SFAr = ST0r + ST2r;
                const float SFAi = ST0i + ST2i;

                const float SFBr = ST1r + ST3i;
                const float SFBi = ST1i - ST3r;

                const uint32_t SA = outputOffset + quarterLen - i;
                const uint32_t SB = outputOffset + halfLen - i;

                fftBuffer[SA] = SFAr;
                fftBuffer[SA + 1] = SFAi;
                fftBuffer[SB] = SFBr;
                fftBuffer[SB + 1] = SFBi;

            }

        }

    }

}

void FFT_completeSpectrum(float *fftBuffer) {

    for (uint32_t i = 2; i < CSIZE >> 1; i += 2) {

        fftBuffer[CSIZE - i] = fftBuffer[i];

        fftBuffer[CSIZE - i + 1] = -fftBuffer[i + 1];

    }

}