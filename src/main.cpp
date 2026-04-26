#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/i2s.h"

#define SAMPLE_RATE 48000
#define AUDIO_BUF_LEN 480
#define FFT_SIZE 512
#define TWO_PI_F 6.28318530717958647692f

#define I2S_MIC_WS GPIO_NUM_5
#define I2S_MIC_SCK GPIO_NUM_4
#define I2S_MIC_SD GPIO_NUM_18

#define I2S_SPK_WS GPIO_NUM_21
#define I2S_SPK_SCK GPIO_NUM_23
#define I2S_SPK_SD GPIO_NUM_22

#define DMA_BUF_COUNT 32
#define SERIAL_BAUD 921600

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

void setupI2SMic() {
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = 512,
    .use_apll = true,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_MIC_SCK,
    .ws_io_num = I2S_MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SD
  };

  ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL));
  ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pin_config));
  ESP_ERROR_CHECK(i2s_zero_dma_buffer(I2S_NUM_0));
  ESP_ERROR_CHECK(i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO));
}

void setupI2SSpeaker() {
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SPK_SCK,
    .ws_io_num = I2S_SPK_WS,
    .data_out_num = I2S_SPK_SD,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL));
  ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_1, &pin_config));
  ESP_ERROR_CHECK(i2s_zero_dma_buffer(I2S_NUM_1));
  ESP_ERROR_CHECK(i2s_set_clk(I2S_NUM_1, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO));
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  setupI2SMic();
  setupI2SSpeaker();
  Serial.println("stereo pipeline ready.");
}

void loop() {
  int32_t input[AUDIO_BUF_LEN * 2];
  float input_norm_mono[AUDIO_BUF_LEN]; // normalized input of xl+xr
  float output_norm_mono[AUDIO_BUF_LEN]; // normalized output
  int16_t output[AUDIO_BUF_LEN];

  size_t bytes_read = 0;
  i2s_read(I2S_NUM_0, input, sizeof(input), &bytes_read, portMAX_DELAY);

  const size_t samples = bytes_read / sizeof(int32_t);
  const size_t frames = samples / 2;

  for (size_t i = 0; i < frames; ++i) {
    
    size_t li = (i << 1);
    size_t ri = li + 1;
    float xl = normalizeMicSample(input[li]);
    float xr = normalizeMicSample(input[ri]);
    input_norm_mono[i] = (xl+xr)*0.5;

  }

  forwardFourierTransform(input, frames); // change here

  // xSpectrum[] contains the frequency-domain buffer here.

  inverseFourierTransform(output, frames); // change here

  for (size_t i = 0; i < frames; ++i){
    output[i] = output_norm_mono
  }

  size_t bytes_written = 0;
  i2s_write(I2S_NUM_1, output, frames * sizeof(int16_t), &bytes_written, portMAX_DELAY);

}
