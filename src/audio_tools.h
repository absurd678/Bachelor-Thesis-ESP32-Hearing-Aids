#pragma once
#include <Arduino.h>
#include "audio_config.h"

struct ComplexSample {
  float real;
  float imag;
};

ComplexSample xSpectrum[FFT_SIZE];
static ComplexSample xTimeDomain[FFT_SIZE];

static inline int16_t clamp16(float y) {
  if (y > 32767.0f) return 32767;
  if (y < -32768.0f) return -32768;
  return (int16_t)lrintf(y);
}

static inline float normalizeMicSample(int32_t value) {
  return (float)(value >> 8) / 8388608.0f;
}

static void fft(ComplexSample *buffer, size_t len, bool inverse) {
  for (size_t i = 1, j = 0; i < len; ++i) {
    size_t bit = len >> 1;
    for (; j & bit; bit >>= 1) {
      j ^= bit;
    }
    j ^= bit;

    if (i < j) {
      const ComplexSample temp = buffer[i];
      buffer[i] = buffer[j];
      buffer[j] = temp;
    }
  }

  for (size_t step = 2; step <= len; step <<= 1) {
    const float angle = (inverse ? TWO_PI_F : -TWO_PI_F) / (float)step;
    const float wStepReal = cosf(angle);
    const float wStepImag = sinf(angle);

    for (size_t block = 0; block < len; block += step) {
      float wReal = 1.0f;
      float wImag = 0.0f;
      const size_t halfStep = step >> 1;

      for (size_t k = 0; k < halfStep; ++k) {
        ComplexSample &even = buffer[block + k];
        ComplexSample &odd = buffer[block + k + halfStep];

        const float oddReal = odd.real * wReal - odd.imag * wImag;
        const float oddImag = odd.real * wImag + odd.imag * wReal;

        odd.real = even.real - oddReal;
        odd.imag = even.imag - oddImag;
        even.real += oddReal;
        even.imag += oddImag;

        const float nextWReal = wReal * wStepReal - wImag * wStepImag;
        wImag = wReal * wStepImag + wImag * wStepReal;
        wReal = nextWReal;
      }
    }
  }

  if (inverse) {
    const float invLen = 1.0f / (float)len;
    for (size_t i = 0; i < len; ++i) {
      buffer[i].real *= invLen;
      buffer[i].imag *= invLen;
    }
  }
}

void forwardFourierTransform(const int32_t *input, size_t frames) {
  const size_t usedFrames = frames < (size_t)AUDIO_BUF_LEN ? frames : (size_t)AUDIO_BUF_LEN;

  for (size_t i = 0; i < FFT_SIZE; ++i) {
    if (i < usedFrames) {
      xSpectrum[i].real = normalizeMicSample(input[i * 2]);
      xSpectrum[i].imag = 0.0f;
    } else {
      xSpectrum[i].real = 0.0f;
      xSpectrum[i].imag = 0.0f;
    }
  }

  fft(xSpectrum, FFT_SIZE, false);
}

void inverseFourierTransform(int16_t *output, size_t frames) {
  for (size_t i = 0; i < FFT_SIZE; ++i) {
    xTimeDomain[i] = xSpectrum[i];
  }

  fft(xTimeDomain, FFT_SIZE, true);

  const size_t usedFrames = frames < (size_t)AUDIO_BUF_LEN ? frames : (size_t)AUDIO_BUF_LEN;
  for (size_t i = 0; i < usedFrames; ++i) {
    output[i] = clamp16(xTimeDomain[i].real * 32767.0f);
  }
}
// ------- imported from myRNN-sim ------------
static void make_hann(float window[FFT_SIZE]) {
    for (int i = 0; i < FFT_SIZE; ++i) {
        window[i] = 0.5f - 0.5f * cosf(TWO_PI_F * i / (float)(FFT_SIZE - 1));
    }
}