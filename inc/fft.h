/****************************************************************************
 * fft.h
 * openacousticdevices.info
 * November 2024
 *****************************************************************************/

#ifndef __FFT_H
#define __FFT_H

#include <stdint.h>

void FFT_realTransform(int16_t *dataBuffer, float *fftBuffer);

void FFT_completeSpectrum(float *fftBuffer);

#endif /* __FFT_H */