
#include <Arduino.h>
#include "driver/i2s_std.h"
#include "driver/gpio.h"
 // LRC - D5
// BCLK - D4
// DIN - D18
// GAIN - GND 
// Vin - 5V

// ---------------- Audio parameters ----------------
#define SAMPLE_RATE      44100
#define BUFFER_FRAMES    32    // number of L/R frames per read/write (adjust as needed)
#define MU 0.00005f // множитель сходимости для LMS фильтра

// Handles
static i2s_chan_handle_t tx_handle = nullptr;
static i2s_chan_handle_t rx_handle = nullptr;

// ---------------- FIR coefficients (stereo band-pass ~300..3400 Hz, example) ----------------
// const float butterworth_lowpass_order3[] = {0.1200, 0.1400, 0.1900, 0.2600, 0.1900, 0.1400, 0.1200};
float w_left[] = { // кэфы для левого киха
  0.1200, 0.1400, 0.1900, 0.2600, 0.1900, 0.1400, 0.1200
};

float w_right[] = { // кэфы для правого киха
  0.1200, 0.1400, 0.1900, 0.2600, 0.1900, 0.1400, 0.1200
};

static const size_t FIR_TAPS = sizeof(w_left)/sizeof(w_left[0]);

// ---------------- Utilities ----------------
static inline int16_t clamp16(float y) {
  if (y > 32767.0f) return 32767;
  if (y < -32768.0f) return -32768;
  return (int16_t)lrintf(y);
}


void recalc_fir(float w_coeffs[FIR_TAPS], int16_t error, int16_t x_signal[BUFFER_FRAMES]){ // файнинг адаптивного фильтра для одного семпла
  for (int i = 0; i < FIR_TAPS; i++){
    w_coeffs[i] += MU*error*x_signal[i];
  }
}


int16_t calc_error_make_y(int16_t input, int16_t prev_val_dir, int16_t prev_val_inv, float w_coeffs[FIR_TAPS]){ // // вх сигнал, прошлый этого же канала, прошлый из противоположного канала для адаптивного фильтра

  static int16_t buffer[FIR_TAPS] = {0}; //  буфер сигнала; инициализируется при первом вызове функции
  memmove(&buffer[1], &buffer[0], (FIR_TAPS - 1) * sizeof(int16_t)); // сдвиг буфера для нового значения
  buffer[0] = input;

  // Вычисление ошибки для фильтров обоих каналов
  int16_t error = input - prev_val_dir;
  
  // Для: Перекрестное складывание сигналов
  int16_t output_ch = input - prev_val_inv; // Ex: левый сигнал - опоздавший отфильтрованный правый

  recalc_fir(w_coeffs, error, buffer);

  return output_ch;
}

int16_t apply_filter(int16_t signal, float w_coeffs[FIR_TAPS]){ // применить фильтр полосовой, отказ от audiotools
  int16_t output = 0;
  for (int i=0; i<FIR_TAPS; i++){
    output += signal*w_coeffs[i];
  }
  return output;
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

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  setupI2S();

  Serial.println("i2s_std + AudioTools::FIR stereo pipeline ready.");
}

void loop() {

  static int16_t input[BUFFER_FRAMES * 2];
  static int16_t output[BUFFER_FRAMES];
  int16_t pBufL[BUFFER_FRAMES] = {0}; // кэш предыдущего сигнала
  int16_t pBufR[BUFFER_FRAMES] = {0}; // кэш предыдущего сигнала
  static int16_t two_channels_buf[2]; // для цикла
  int16_t error_array_left_fir[FIR_TAPS]; // ошибка для левого фильтра
  int16_t error_array_right_fir[FIR_TAPS]; // ошибка для правого фильтра

  size_t bytes_read = 0;
  if (i2s_channel_read(rx_handle, input, sizeof(input), &bytes_read, 1000) != ESP_OK || bytes_read == 0) return;

  size_t samples = bytes_read / sizeof(int16_t);
  size_t frames  = samples / 2;

  // stereo per-sample filtering
  for (size_t i = 0; i < frames; ++i) {
    size_t li = (i << 1);
    size_t ri = li + 1;
  // TODO: почему кэфы флоат, а буфер значений - инт?!?!!
    int16_t yl = calc_error_make_y(input[li], pBufL[i], pBufR[i], w_left); // для левого канала 
    int16_t yr = calc_error_make_y(input[ri], pBufR[i], pBufL[i], w_right); // для правого канала 
     
    // Запись результатов
    output[i] = yl+yr; // 

    // ФИЛЬТРАЦИЯ
    pBufL[i] = apply_filter(yl, w_left);
    pBufR[i] = apply_filter(yr, w_right);
  }

  size_t bytes_written = 0;
  i2s_channel_write(tx_handle, output, frames * sizeof(int16_t), &bytes_written, 1000);
  
}
