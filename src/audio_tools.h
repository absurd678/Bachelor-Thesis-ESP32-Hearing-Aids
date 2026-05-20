#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "audio_config.h"

struct ComplexSample {
  float real;
  float imag;
};

static inline int16_t clamp16(float y) {
  if (y > 32767.0f) return 32767;
  if (y < -32768.0f) return -32768;
  return (int16_t)lrintf(y);
}

static inline float normalizeMicSample(int32_t value) {
  return (float)(value >> 8) / 8388608.0f;
}

static void fft(ComplexSample *buffer, size_t n, bool inverse) {
  for (int i = 1, j = 0; i < n; ++i) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;

        if (i < j) {
            const ComplexSample tmp = buffer[i];
            buffer[i] = buffer[j];
            buffer[j] = tmp;
        }
    }

    for (int len = 2; len <= n; len <<= 1) {
        float angle = TWO_PI_F / (float)len;
        if (!inverse) {
            angle = -angle;
        }

        const float wlen_re = cosf(angle);
        const float wlen_im = sinf(angle);

        for (int i = 0; i < n; i += len) {
            float w_re = 1.0f;
            float w_im = 0.0f;

            for (int j = 0; j < len / 2; ++j) {
                const ComplexSample u = buffer[i + j];
                ComplexSample v;
                v.real = buffer[i + j + len / 2].real * w_re - buffer[i + j + len / 2].imag * w_im;
                v.imag = buffer[i + j + len / 2].real * w_im + buffer[i + j + len / 2].imag * w_re;

                buffer[i + j].real = u.real + v.real;
                buffer[i + j].imag = u.imag + v.imag;
                buffer[i + j + len / 2].real = u.real - v.real;
                buffer[i + j + len / 2].imag = u.imag - v.imag;

                const float next_re = w_re * wlen_re - w_im * wlen_im;
                const float next_im = w_re * wlen_im + w_im * wlen_re;
                w_re = next_re;
                w_im = next_im;
            }
        }
    }

    if (inverse) {
        for (int i = 0; i < n; ++i) {
            buffer[i].real /= (float)n;
            buffer[i].imag /= (float)n;
        }
    }
  }


// ------- imported from myRNN-sim ------------
static void make_hann(float window[FFT_SIZE]) {
    for (int i = 0; i < FFT_SIZE; ++i) {
        window[i] = 0.5f - 0.5f * cosf(TWO_PI_F * i / (float)(FFT_SIZE - 1));
    }
}
