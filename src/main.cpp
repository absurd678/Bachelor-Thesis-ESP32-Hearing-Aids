#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "lms_filters.h"

#define SAMPLE_RATE 48000
#define AUDIO_BUF_LEN 32
#define FILTER_ORDER 10
#define MU 0.00005f

#define I2S_MIC_WS GPIO_NUM_5
#define I2S_MIC_SCK GPIO_NUM_4
#define I2S_MIC_SD GPIO_NUM_18

#define I2S_SPK_WS GPIO_NUM_21
#define I2S_SPK_SCK GPIO_NUM_23
#define I2S_SPK_SD GPIO_NUM_22

#define DMA_BUF_COUNT 32
#define SERIAL_BAUD 921600
#define REC_DOWNSAMPLE 3
#define REC_SAMPLE_RATE (SAMPLE_RATE / REC_DOWNSAMPLE)
#define DEBUG_PLOT 0

static const char* TAG = "I2S_AUDIO";
static constexpr float GAIN = 1.0f;
static bool recording = false;

LMSFilter left_filter(FILTER_ORDER, MU);
LMSFilter right_filter(FILTER_ORDER, MU);

static inline int16_t clamp16(float y) {
  if (y > 32767.0f) return 32767;
  if (y < -32768.0f) return -32768;
  return (int16_t)lrintf(y);
}

static inline float normalizeMicSample(int32_t value) {
  return (float)(value >> 8) / 8388608.0f;
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
  int16_t output[AUDIO_BUF_LEN];

  size_t bytes_read = 0;
  i2s_read(I2S_NUM_0, input, sizeof(input), &bytes_read, portMAX_DELAY);

#if DEBUG_PLOT
  if (bytes_read != sizeof(input)) {
    ESP_LOGW(TAG, "Short read: got %d expected %d", bytes_read, sizeof(input));
  }
#endif

  const size_t samples = bytes_read / sizeof(int32_t);
  const size_t frames = samples / 2;

  for (size_t i = 0; i < frames; ++i) {
    const size_t li = i * 2;
    const size_t ri = li + 1;

    const float xl = normalizeMicSample(input[li]);
    const float xr = normalizeMicSample(input[ri]);

#if DEBUG_PLOT
    static int plot_counter = 0;
    if (++plot_counter % 4 == 0) {
      Serial.print(">mic_left:");
      Serial.println(input[li], 4);
      Serial.print(">mic_right:");
      Serial.println(input[ri], 4);
    }
#endif

    // Cross-reference NLMS: each channel uses the opposite mic as reference.
    const float yl = left_filter.process(xr, xl);
    const float yr = right_filter.process(xl, xr);
    const float result = (yl + yr) * 0.5f * GAIN;
    const int16_t sample_out = clamp16(result * 32767.0f);

    output[li] = sample_out;
    output[ri] = sample_out;

#if DEBUG_PLOT
    static int plot_counter1 = 0;
    if (++plot_counter1 % 4 == 0) {
      Serial.print(">output:");
      Serial.println(sample_out);
    }
#endif
  }

  size_t bytes_written = 0;
  i2s_write(I2S_NUM_1, output, frames * sizeof(int16_t), &bytes_written, portMAX_DELAY);

  while (Serial.available()) {
    const char cmd = Serial.read();
    if (cmd == 'R' && !recording) {
      recording = true;
      const uint8_t marker[] = {0xAA, 0x55};
      Serial.write(marker, sizeof(marker));
    } else if (cmd == 'S' && recording) {
      recording = false;
      const uint8_t marker[] = {0x55, 0xAA};
      Serial.write(marker, sizeof(marker));
    }
  }

  if (recording) {
    for (size_t i = 0; i < frames; i += REC_DOWNSAMPLE) {
      Serial.write((uint8_t*)&output[i * 2], sizeof(int16_t));
    }
  }
}
