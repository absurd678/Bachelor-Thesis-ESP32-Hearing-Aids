
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

static float fir_coef[] = {
  0.1200, 0.1400, 0.1900, 0.2600, 0.1900, 0.1400, 0.1200
};

// LRC - D5
// BCLK - D4
// DIN - D18
// GAIN - GND 
// Vin - 5V
static const size_t FIR_TAPS = sizeof(fir_coef)/sizeof(fir_coef[0]);
// const float GAIN = 1.0f; // потом можно менять
// // Two FIR instances: left/right
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


int16_t pBufL[BUFFER_FRAMES] = {0}; // кэш предыдущего сигнала
int16_t pBufR[BUFFER_FRAMES] = {0}; // кэш предыдущего сигнала
// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  setupI2S();

  Serial.println("i2s_std + AudioTools::FIR stereo pipeline ready.");
}

void loop() {

  static int16_t inBuf[BUFFER_FRAMES * 2];
  
  static int16_t outBuf[BUFFER_FRAMES];
  static int16_t two_channels_buf[2]; // для цикла

  size_t bytes_read = 0;
  if (i2s_channel_read(rx_handle, inBuf, sizeof(inBuf), &bytes_read, 1000) != ESP_OK || bytes_read == 0) return;

  size_t samples = bytes_read / sizeof(int16_t);
  size_t frames  = samples / 2;

  // stereo per-sample filtering
  for (size_t i = 0; i < frames; ++i) {
    size_t li = (i << 1);
    size_t ri = li + 1;

    // Достаем левый и правый канал
    int16_t sl = inBuf[li];
    int16_t sr = inBuf[ri];

    // Вычисление ошибки для фильтров обоих каналов TODO: сделать чтоб использовалось
    int16_t errl = sl - clamp16(pBufL[i]);
    int16_t errr = sr - clamp16(pBufR[i]);

    // Перекрестное складывание сигналов
    int16_t yl = sl - clamp16(pBufR[i]); // левый сигнал - опоздавший отфильтрованный правый 
    int16_t yr = sr - clamp16(pBufL[i]); // наоборот 
    // Запись результатов
    outBuf[i] = (yl)+(yr);

    // Для фильтрации левого и правого

    pBufL[i] = firL.process(sl);
    pBufR[i] = firR.process(sr);
  }

  size_t bytes_written = 0;
  i2s_channel_write(tx_handle, outBuf, frames * sizeof(int16_t), &bytes_written, 1000);
  
}
