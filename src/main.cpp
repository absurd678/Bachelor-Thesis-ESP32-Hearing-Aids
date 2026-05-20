#include <cstdint>
#include <cstdio>
#include <limits>

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "audio_config.h"
#include "audio_tools.h"
#include "lms_filters.h"
#include "rnn_ops.h"

static const char* TAG = "hearing_aids";
static void checkEsp(esp_err_t err, const char* step) {
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s failed: %s (%d)", step, esp_err_to_name(err), (int)err);
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
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
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = I2S_MIC_SCK,
    .ws_io_num = I2S_MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SD
  };

  checkEsp(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, nullptr), "mic i2s_driver_install");
  checkEsp(i2s_set_pin(I2S_NUM_0, &pin_config), "mic i2s_set_pin");
  checkEsp(i2s_zero_dma_buffer(I2S_NUM_0), "mic i2s_zero_dma_buffer");
  checkEsp(i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO), "mic i2s_set_clk");
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
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = I2S_SPK_SCK,
    .ws_io_num = I2S_SPK_WS,
    .data_out_num = I2S_SPK_SD,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  checkEsp(i2s_driver_install(I2S_NUM_1, &i2s_config, 0, nullptr), "speaker i2s_driver_install");
  checkEsp(i2s_set_pin(I2S_NUM_1, &pin_config), "speaker i2s_set_pin");
  checkEsp(i2s_zero_dma_buffer(I2S_NUM_1), "speaker i2s_zero_dma_buffer");
  checkEsp(i2s_set_clk(I2S_NUM_1, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO), "speaker i2s_set_clk");
}

// -------------- GLOBAL VARIABLES ---------------
int32_t input[AUDIO_BUF_LEN * 2];
int16_t output[kMaxRealtimeOutput];
float filter_block[AUDIO_BUF_LEN];

LMSFilter left_filter(TAPS, MU);
LMSFilter right_filter(TAPS, MU);
float left_delay = 0;
float right_delay = 0;
RealtimeRnnFilter rnn_filter;

static int16_t micToPcm16(int32_t sample) {
  return clamp16(normalizeMicSample(sample) * 32767.0f);
}

static void setOutputFrame(size_t frame, int16_t sample) {
  output[frame] = sample;
}

static void logAudioStats(size_t frames, size_t bytes_read, size_t bytes_written) {
#if AUDIO_DEBUG_STATS
  static uint32_t blocks = 0;
  if (++blocks < 200) {
    return;
  }
  blocks = 0;

  int32_t min_l = std::numeric_limits<int32_t>::max();
  int32_t max_l = std::numeric_limits<int32_t>::min();
  int32_t min_r = std::numeric_limits<int32_t>::max();
  int32_t max_r = std::numeric_limits<int32_t>::min();
  int16_t min_o = std::numeric_limits<int16_t>::max();
  int16_t max_o = std::numeric_limits<int16_t>::min();

  for (size_t i = 0; i < frames; ++i) {
    const size_t li = i << 1;
    const size_t ri = li + 1;
    if (input[li] < min_l) min_l = input[li];
    if (input[li] > max_l) max_l = input[li];
    if (input[ri] < min_r) min_r = input[ri];
    if (input[ri] > max_r) max_r = input[ri];
    const int16_t out = output[i];
    if (out < min_o) min_o = out;
    if (out > max_o) max_o = out;
  }

  ESP_LOGI(TAG,
           "audio frames=%u read=%u written=%u inL=[%ld,%ld] inR=[%ld,%ld] out=[%d,%d]",
           (unsigned)frames,
           (unsigned)bytes_read,
           (unsigned)bytes_written,
           (long)min_l,
           (long)max_l,
           (long)min_r,
           (long)max_r,
           (int)min_o,
           (int)max_o);
#endif
}

#if AUDIO_MODE == AUDIO_MODE_TEST_TONE
static void writeTestTone() {
  static float phase = 0.0f;
  const float phase_step = TWO_PI_F * AUDIO_TEST_TONE_HZ / (float)SAMPLE_RATE;

  for (size_t i = 0; i < AUDIO_BUF_LEN; ++i) {
    setOutputFrame(i, clamp16(sinf(phase) * AUDIO_TEST_TONE_GAIN * 32767.0f));
    phase += phase_step;
    if (phase >= TWO_PI_F) {
      phase -= TWO_PI_F;
    }
  }

  size_t bytes_written = 0;
  checkEsp(i2s_write(I2S_NUM_1, output, AUDIO_BUF_LEN * sizeof(int16_t), &bytes_written, portMAX_DELAY),
           "speaker test tone i2s_write");
}
#endif

static void processAudio() {
#if AUDIO_MODE == AUDIO_MODE_TEST_TONE
  writeTestTone();
  return;
#else
  // ------------ i2s read ---------------
  size_t bytes_read = 0;
  checkEsp(i2s_read(I2S_NUM_0, input, sizeof(input), &bytes_read, portMAX_DELAY), "mic i2s_read");
  
  const size_t samples = bytes_read / sizeof(int32_t);
  const size_t frames = samples / 2;

  for (size_t i = 0; i < frames; ++i) {
    size_t li = (i << 1);

#if AUDIO_MODE == AUDIO_MODE_PASSTHROUGH
    setOutputFrame(i, micToPcm16(input[li]));
#elif AUDIO_MODE == AUDIO_MODE_LMS
    size_t ri = li + 1;
    float xl = normalizeMicSample(input[li]);
    float xr = normalizeMicSample(input[ri]);
    
    // const float xld = left_delay;
    // const float xrd = right_delay;

    // const float p1 = left_filter.process(xl, xld);
    // const float rr = xrd - p1;

    // const float p2 = right_filter.process(xr, xrd);
    // const float rl = xld - p2;

    filter_block[i] = 0.5f * (xl + xr);

    // left_delay = xl;
    // right_delay = xr;

    //setOutputFrame(i, clamp16(filter_block[i] * 32767.0f));
#endif
  }
  
  const size_t output_samples = rnn_filter.process(
    filter_block,
    frames,
    output,
    kMaxRealtimeOutput
  );

  size_t bytes_written = 0;
  // if (output_samples > 0) {
    checkEsp(i2s_write(I2S_NUM_1, output, output_samples * sizeof(int16_t), &bytes_written, portMAX_DELAY),
             "speaker i2s_write");
    logAudioStats(frames, bytes_read, bytes_written);
    
  // }
#endif
}

static void audioTask(void*) {
  while (true) {
    processAudio();
  }
}

extern "C" void app_main(void) {
  vTaskDelay(pdMS_TO_TICKS(200));

  ESP_LOGI(TAG, "PSRAM size: %zu", heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
  ESP_LOGI(TAG, "Free PSRAM: %zu", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

  setupI2SMic();
  setupI2SSpeaker();

  ESP_LOGI(TAG, "stereo pipeline ready.");
  ESP_LOGI(TAG, "audio mode: %d", AUDIO_MODE);
  rnn_filter.init();

  xTaskCreatePinnedToCore(audioTask, "audio", 8192, nullptr, 5, nullptr, 1);
}
