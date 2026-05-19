#include <cstdint>
#include <cstdio>
#include <limits>

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "audio_config.h"
#include "audio_tools.h"
#include "lms_filters.h"
//#include "rnn_ops.h"

static const char* TAG = "hearing_aids";
static i2s_chan_handle_t mic_rx_chan = nullptr;
static i2s_chan_handle_t spk_tx_chan = nullptr;

static void checkEsp(esp_err_t err, const char* step) {
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s failed: %s (%d)", step, esp_err_to_name(err), (int)err);
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void setupI2SMic() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = I2S_MIC_SCK,
      .ws = I2S_MIC_WS,
      .dout = I2S_GPIO_UNUSED,
      .din = I2S_MIC_SD,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };

  checkEsp(i2s_new_channel(&chan_cfg, nullptr, &mic_rx_chan), "mic i2s_new_channel");
  checkEsp(i2s_channel_init_std_mode(mic_rx_chan, &std_cfg), "mic i2s_channel_init_std_mode");
  checkEsp(i2s_channel_enable(mic_rx_chan), "mic i2s_channel_enable");
}

void setupI2SSpeaker() {
  
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);

  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = I2S_SPK_SCK,
      .ws = I2S_SPK_WS,
      .dout = I2S_SPK_SD,
      .din = I2S_GPIO_UNUSED,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };
  //std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  checkEsp(i2s_new_channel(&chan_cfg, &spk_tx_chan, nullptr), "speaker i2s_new_channel");
  checkEsp(i2s_channel_init_std_mode(spk_tx_chan, &std_cfg), "speaker i2s_channel_init_std_mode");
  checkEsp(i2s_channel_enable(spk_tx_chan), "speaker i2s_channel_enable");
}

// -------------- GLOBAL VARIABLES ---------------
int32_t input[AUDIO_BUF_LEN * 2];
int16_t output[AUDIO_BUF_LEN];//kMaxRealtimeOutput];
float filter_block[AUDIO_BUF_LEN];

LMSFilter left_filter(TAPS, MU);
LMSFilter right_filter(TAPS, MU);
float left_delay = 0;
float right_delay = 0;
// RealtimeRnnFilter rnn_filter;

static int16_t micToPcm16(int32_t sample) {
  return clamp16(normalizeMicSample(sample) * 32767.0f);
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
    if (output[i] < min_o) min_o = output[i];
    if (output[i] > max_o) max_o = output[i];
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
    output[i] = clamp16(sinf(phase) * AUDIO_TEST_TONE_GAIN * 32767.0f);
    phase += phase_step;
    if (phase >= TWO_PI_F) {
      phase -= TWO_PI_F;
    }
  }

  size_t bytes_written = 0;
  checkEsp(i2s_channel_write(spk_tx_chan, output, sizeof(output), &bytes_written, portMAX_DELAY),
           "speaker test tone i2s_channel_write");
}
#endif

static void processAudio() {
#if AUDIO_MODE == AUDIO_MODE_TEST_TONE
  writeTestTone();
  return;
#else
  // ------------ i2s read ---------------
  size_t bytes_read = 0;
  checkEsp(i2s_channel_read(mic_rx_chan, input, sizeof(input), &bytes_read, portMAX_DELAY), "mic i2s_channel_read");
  
  const size_t samples = bytes_read / sizeof(int32_t);
  const size_t frames = samples / 2;

  for (size_t i = 0; i < frames; ++i) {
    size_t li = (i << 1);

#if AUDIO_MODE == AUDIO_MODE_PASSTHROUGH
    output[i] = micToPcm16(input[li]);
#elif AUDIO_MODE == AUDIO_MODE_LMS
    size_t ri = li + 1;
    float xl = normalizeMicSample(input[li]);
    float xr = normalizeMicSample(input[ri]);
    
    const float xld = left_delay;
    const float xrd = right_delay;

    const float p1 = left_filter.process(xl, xld);
    const float rr = xrd - p1;

    const float p2 = right_filter.process(xr, xrd);
    const float rl = xld - p2;

    filter_block[i] = 0.5f * (rl + rr);

    left_delay = xl;
    right_delay = xr;

    output[i] = clamp16(filter_block[i] * 32767.0f);
#endif
  }
  
  // const size_t output_samples = rnn_filter.process(
  //   filter_block,
  //   frames,
  //   output,
  //   kMaxRealtimeOutput
  // );

  size_t bytes_written = 0;
  // if (output_samples > 0) {
    checkEsp(i2s_channel_write(spk_tx_chan, output, frames * sizeof(int16_t), &bytes_written, portMAX_DELAY),
             "speaker i2s_channel_write");
    logAudioStats(frames, bytes_read, bytes_written);
    
  // }
#endif
}

static void audioTask(void*) {
  while (true) {
    processAudio();
    vTaskDelay(1);
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
  //rnn_filter.init();

  xTaskCreatePinnedToCore(audioTask, "audio", 8192, nullptr, 5, nullptr, 1);
}
