#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "audio_config.h"
#include "audio_tools.h"
#include "lms_filters.h"
#include "rnn_ops.h"

static void checkEsp(esp_err_t err, const char* step) {
  if (err != ESP_OK) {
    Serial.printf("%s failed: %s (%d)\n", step, esp_err_to_name(err), (int)err);
    while (true) {
      delay(1000);
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
    .bck_io_num = I2S_MIC_SCK,
    .ws_io_num = I2S_MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SD
  };

  checkEsp(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL), "mic i2s_driver_install");
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
    .bck_io_num = I2S_SPK_SCK,
    .ws_io_num = I2S_SPK_WS,
    .data_out_num = I2S_SPK_SD,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  checkEsp(i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL), "speaker i2s_driver_install");
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

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  
  setupI2SMic();
  setupI2SSpeaker();
  rnn_filter.init();

  Serial.println("stereo pipeline ready.");


}

void loop() {
  
  // ------------ i2s read ---------------
  size_t bytes_read = 0;
  i2s_read(I2S_NUM_0, input, sizeof(input), &bytes_read, portMAX_DELAY);

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
    i2s_write(I2S_NUM_1, output, output_samples * sizeof(int16_t), &bytes_written, portMAX_DELAY);
  }

}
