
#include <Arduino.h>
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "AudioTools.h"   // using only FIR<float>

// ---------------- I2S pinout (matches your earlier setup) ----------------
#define I2S_WS    25   // LRCLK / WS
#define I2S_SD    33   // DIN  (mic -> ESP32)
#define I2S_SCK   32   // BCLK
#define I2S_DOUT  35   // DOUT (ESP32 -> amp/DAC)

// ---------------- Audio parameters ----------------
#define SAMPLE_RATE      44100
#define BUFFER_FRAMES    32    // number of L/R frames per read/write (adjust as needed)

// Handles
static i2s_chan_handle_t tx_handle = nullptr;
static i2s_chan_handle_t rx_handle = nullptr;

// ---------------- FIR coefficients (stereo band-pass ~300..3400 Hz, example) ----------------
static float fir_coef[] = {  1.0f 
  // -0.001312f, -0.002066f, -0.001226f,  0.001834f,  0.005255f,  0.006243f,
  //  0.002857f, -0.004488f, -0.012244f, -0.014256f, -0.006573f,  0.010656f,
  //  0.029334f,  0.039857f,  0.033691f,  0.008575f, -0.027557f, -0.059848f,
  // -0.073241f, -0.055848f,  0.000000f,  0.083827f,  0.177845f,  0.250000f,
  //  0.277845f,  0.250000f,  0.177845f,  0.083827f,  0.000000f, -0.055848f,
  // -0.073241f, -0.059848f, -0.027557f,  0.008575f,  0.033691f,  0.039857f,
  //  0.029334f,  0.010656f, -0.006573f, -0.014256f, -0.012244f, -0.004488f,
  //  0.002857f,  0.006243f,  0.005255f,  0.001834f, -0.001226f, -0.002066f,
  // -0.001312f
};
static const size_t FIR_TAPS = sizeof(fir_coef)/sizeof(fir_coef[0]);

// Two FIR instances: left/right
static FIR<float> firL(fir_coef);
static FIR<float> firR(fir_coef);

// ---------------- Utilities ----------------
static inline int16_t clamp16(float y) {
  if (y > 32767.0f) return 32767;
  if (y < -32768.0f) return -32768;
  return (int16_t)lrintf(y);
}

// ---------------- I2S setup using ESP-IDF std driver ----------------
static void setupI2S() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));

  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)GPIO_NUM_4,
      .ws   = (gpio_num_t)GPIO_NUM_5,
      .dout = (gpio_num_t)GPIO_NUM_18,
      .din  = (gpio_num_t)GPIO_NUM_19,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv   = false,
      }
    },
  };

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
  ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  setupI2S();

  Serial.print("FIR taps: "); Serial.println(FIR_TAPS);
  Serial.println("i2s_std + AudioTools::FIR stereo pipeline ready.");
}

void loop() {
  static int16_t inBuf[BUFFER_FRAMES * 2];
  static int16_t outBuf[BUFFER_FRAMES * 2];

  size_t bytes_read = 0;
  if (i2s_channel_read(rx_handle, inBuf, sizeof(inBuf), &bytes_read, 1000) != ESP_OK || bytes_read == 0) return;

  size_t samples = bytes_read / sizeof(int16_t);
  size_t frames  = samples / 2;

  // stereo per-sample filtering
  for (size_t i = 0; i < frames; ++i) {
    size_t li = (i << 1);
    size_t ri = li + 1;

    float xl = (float)inBuf[li];
    float xr = (float)inBuf[ri];

    float yl = firL.process(xl);
    float yr = firR.process(xr);

    outBuf[li] = clamp16(yl);
    outBuf[ri] = clamp16(yr);
  }

  size_t bytes_written = 0;
  i2s_channel_write(tx_handle, outBuf, sizeof(outBuf), &bytes_written, 1000);
}
