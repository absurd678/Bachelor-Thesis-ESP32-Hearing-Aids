
#include <Arduino.h>
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "AudioTools.h"   // using only FIR<float>

// ---------------- Audio parameters ----------------
#define SAMPLE_RATE      44100
#define BUFFER_FRAMES    32    // number of L/R frames per read/write (adjust as needed)

// Handles
static i2s_chan_handle_t tx_handle = nullptr;
static i2s_chan_handle_t rx_handle = nullptr;

// ---------------- FIR coefficients (stereo band-pass ~300..3400 Hz, example) ----------------
static float fir_coef[] = { 1.0f
    //   -0.000490803f,
    // -0.000600824f,
    // -0.000628649f,
    // -0.000566798f,
    // -0.000423759f,
    // -0.000226194f,
    // -0.000018559f,
    // 0.000141501f,
    // 0.000192840f,
    // 0.000084908f,
    // -0.000206416f,
    // -0.000665488f,
    // -0.001230973f,
    // -0.001801309f,
    // -0.002253072f,
    // -0.002470122f,
    // -0.002377571f,
    // -0.001971781f,
    // -0.001336806f,
    // -0.000639670f,
    // -0.000101474f,
    // 0.000052272f,
    // -0.000347844f,
    // -0.001358684f,
    // -0.002888489f,
    // -0.004694729f,
    // -0.006422276f,
    // -0.007678314f,
    // -0.008129791f,
    // -0.007600933f,
    // -0.006144823f,
    // -0.004066089f,
    // -0.001881280f,
    // -0.000217998f,
    // 0.000329994f,
    // -0.000639423f,
    // -0.003204746f,
    // -0.007052101f,
    // -0.011496077f,
    // -0.015597292f,
    // -0.018360513f,
    // -0.018974949f,
    // -0.017042989f,
    // -0.012739706f,
    // -0.006855051f,
    // -0.000692996f,
    // 0.004167197f,
    // 0.006209332f,
    // 0.004352484f,
    // -0.001723869f,
    // -0.011380694f,
    // -0.023005138f,
    // -0.034204947f,
    // -0.042192114f,
    // -0.044293738f,
    // -0.038495899f,
    // -0.023912116f,
    // -0.001075922f,
    // 0.028013275f,
    // 0.060115905f,
    // 0.091221091f,
    // 0.117180555f,
    // 0.134394123f,
    // 0.140419908f,
    // 0.134394123f,
    // 0.117180555f,
    // 0.091221091f,
    // 0.060115905f,
    // 0.028013275f,
    // -0.001075922f,
    // -0.023912116f,
    // -0.038495899f,
    // -0.044293738f,
    // -0.042192114f,
    // -0.034204947f,
    // -0.023005138f,
    // -0.011380694f,
    // -0.001723869f,
    // 0.004352484f,
    // 0.006209332f,
    // 0.004167197f,
    // -0.000692996f,
    // -0.006855051f,
    // -0.012739706f,
    // -0.017042989f,
    // -0.018974949f,
    // -0.018360513f,
    // -0.015597292f,
    // -0.011496077f,
    // -0.007052101f,
    // -0.003204746f,
    // -0.000639423f,
    // 0.000329994f,
    // -0.000217998f,
    // -0.001881280f,
    // -0.004066089f,
    // -0.006144823f,
    // -0.007600933f,
    // -0.008129791f,
    // -0.007678314f,
    // -0.006422276f,
    // -0.004694729f,
    // -0.002888489f,
    // -0.001358684f,
    // -0.000347844f,
    // 0.000052272f,
    // -0.000101474f,
    // -0.000639670f,
    // -0.001336806f,
    // -0.001971781f,
    // -0.002377571f,
    // -0.002470122f,
    // -0.002253072f,
    // -0.001801309f,
    // -0.001230973f,
    // -0.000665488f,
    // -0.000206416f,
    // 0.000084908f,
    // 0.000192840f,
    // 0.000141501f,
    // -0.000018559f,
    // -0.000226194f,
    // -0.000423759f,
    // -0.000566798f,
    // -0.000628649f,
    // -0.000600824f,
    // -0.000490803f
};

// LRC - D5
// BCLK - D4
// DIN - D18
// GAIN - GND 
// Vin - 5V
static const size_t FIR_TAPS = sizeof(fir_coef)/sizeof(fir_coef[0]);
const float GAIN = 1.0f; // потом можно менять
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
      .dout = (gpio_num_t)GPIO_NUM_18,  // din for max, sd for inmp441
      .din  = (gpio_num_t)GPIO_NUM_19, // not used
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

// // ---------------- Arduino ----------------
// void setup() {
//   Serial.begin(115200);
//   delay(200);
//   setupI2S();

//   Serial.print("FIR taps: "); Serial.println(FIR_TAPS);
//   Serial.println("i2s_std + AudioTools::FIR stereo pipeline ready.");
// }

// //static bool printed = false;

// void loop() {
//   static int16_t inBuf[BUFFER_FRAMES * 2];
//   static int16_t outBuf[BUFFER_FRAMES * 2];

//   size_t bytes_read = 0;
//   if (i2s_channel_read(rx_handle, inBuf, sizeof(inBuf), &bytes_read, 1000) != ESP_OK || bytes_read == 0) return;

//   size_t samples = bytes_read / sizeof(int16_t);
//   size_t frames  = samples / 2;

//   // stereo per-sample filtering
//   for (size_t i = 0; i < frames; ++i) {
//     size_t li = (i << 1);
//     size_t ri = li + 1;

//     float xl = (float)inBuf[li];
//     float xr = (float)inBuf[ri];

//     float yl = firL.process(xl)* GAIN;
//     float yr = firR.process(xr)* GAIN;
//     int y_result = clamp16(yl) - clamp16(yr);
//     outBuf[i] = y_result;
//     // outBuf[li] = clamp16(yl);
//     // outBuf[ri] = clamp16(yr);
//   }

//   size_t bytes_written = 0;
//   i2s_channel_write(tx_handle, outBuf, frames *2* sizeof(int16_t), &bytes_written, 1000);
// }
