#include <cstdint>
#include <cstdio>

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
#include "rnn_ops.h"

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
  i2s_chan_config_t chan_cfg = {
    .id = I2S_NUM_0,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = DMA_BUF_COUNT,
    .dma_frame_num = 512,
    .auto_clear = true,
    .intr_priority = 1,
  };

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
  std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;

  checkEsp(i2s_new_channel(&chan_cfg, nullptr, &mic_rx_chan), "mic i2s_new_channel");
  checkEsp(i2s_channel_init_std_mode(mic_rx_chan, &std_cfg), "mic i2s_channel_init_std_mode");
  checkEsp(i2s_channel_enable(mic_rx_chan), "mic i2s_channel_enable");
}

void setupI2SSpeaker() {
  i2s_chan_config_t chan_cfg = {
    .id = I2S_NUM_1,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = DMA_BUF_COUNT,
    .dma_frame_num = 512,
    .auto_clear = true,
    .intr_priority = 1,
  };

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
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  checkEsp(i2s_new_channel(&chan_cfg, &spk_tx_chan, nullptr), "speaker i2s_new_channel");
  checkEsp(i2s_channel_init_std_mode(spk_tx_chan, &std_cfg), "speaker i2s_channel_init_std_mode");
  checkEsp(i2s_channel_enable(spk_tx_chan), "speaker i2s_channel_enable");
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

static void processAudio() {
  // ------------ i2s read ---------------
  size_t bytes_read = 0;
  checkEsp(i2s_channel_read(mic_rx_chan, input, sizeof(input), &bytes_read, portMAX_DELAY), "mic i2s_channel_read");
  
  const size_t samples = bytes_read / sizeof(int32_t);
  const size_t frames = samples / 2;

  // -------------- lms processing -----------------
  for (size_t i = 0; i < frames; ++i) {
    
    size_t li = (i << 1);
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
  }

  const size_t output_samples = rnn_filter.process(
    filter_block,
    frames,
    output,
    kMaxRealtimeOutput
  );

  size_t bytes_written = 0;
  if (output_samples > 0) {
    checkEsp(i2s_channel_write(spk_tx_chan, output, output_samples * sizeof(int16_t), &bytes_written, portMAX_DELAY),
             "speaker i2s_channel_write");
  }
}

extern "C" void app_main(void) {
  vTaskDelay(pdMS_TO_TICKS(200));

  ESP_LOGI(TAG, "PSRAM size: %zu", heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
  ESP_LOGI(TAG, "Free PSRAM: %zu", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

  setupI2SMic();
  setupI2SSpeaker();

  ESP_LOGI(TAG, "stereo pipeline ready.");
  rnn_filter.init();

  while (true) {
    processAudio();
  }
}
