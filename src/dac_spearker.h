
#include <Arduino.h>
#include "driver/i2s.h"
#include "driver/gpio.h"

#ifndef DAC_SPEARKER_H
#define DAC_SPEARKER_H
#endif
// Audio parameters
#define SAMPLE_RATE      44100
#define BUFFER_FRAMES    32    // number of L/R frames per read/write (adjust as needed)
#define MU 0.00005f // множитель сходимости для LMS фильтра
//пины для INMP44
#define I2S_MIC_WS GPIO_NUM_25
#define I2S_MIC_SCK GPIO_NUM_32
#define I2S_MIC_SD GPIO_NUM_33
//пины для MAX98357A
#define I2S_SPK_WS GPIO_NUM_25
#define I2S_SPK_SCK GPIO_NUM_32
#define I2S_SPK_SD GPIO_NUM_33
#define bufer_i2s_lenb 512
 // LRC - I2S_SPK_WS -- соответствие пинов для справки
// BCLK - I2S_SPK_SCK
// DIN - I2S_SPK_SD
// GAIN - GND 
// Vin - 5V

// ----------------- Global variables ---------------------------
// FIR coefficients (stereo band-pass ~300..3400 Hz, example)
float w_left[] = { // left
  0.1200, 0.1400, 0.1900, 0.2600, 0.1900, 0.1400, 0.1200
};

float w_right[] = { // right
  0.1200, 0.1400, 0.1900, 0.2600, 0.1900, 0.1400, 0.1200
};

static const size_t FILTER_ORDER = sizeof(w_left)/sizeof(w_left[0]);

float pBufL[BUFFER_FRAMES] = {0}; // filtered signal saved
float pBufR[BUFFER_FRAMES] = {0}; // filtered signal saved
float error_array_left_fir[FILTER_ORDER]; // left filter error
float error_array_right_fir[FILTER_ORDER]; // right filter error



// ---------------- Utilities ----------------
static inline int16_t clamp16(float y) {
  if (y > 32767.0f) return 32767;
  if (y < -32768.0f) return -32768;
  return (int16_t)lrintf(y);
}

float normalize(int32_t value) { // to make int32 be in the range of -1, 1
  return (float)(value >> 14)/8192.0f;
}

int16_t normalize_speaker(float value){ // normalize a value for max chip
  return clamp16((int16_t)(value*32767.0f*SAMPLE_RATE));
}

void recalc_fir(float w_coeffs[FILTER_ORDER], float error, float x_signal[BUFFER_FRAMES]){ // файнинг адаптивного фильтра для одного семпла
  for (int i = 0; i < FILTER_ORDER; i++){
    w_coeffs[i] += MU*error*x_signal[i];
  }
}


int16_t calc_error_make_y(float input, float prev_val_dir, float prev_val_inv, float w_coeffs[FILTER_ORDER]){ // // вх сигнал, прошлый этого же канала, прошлый из противоположного канала для адаптивного фильтра

  static float buffer[FILTER_ORDER] = {0}; //  буфер сигнала; инициализируется при первом вызове функции
  memmove(&buffer[1], &buffer[0], (FILTER_ORDER - 1) * sizeof(float)); // сдвиг буфера для нового значения
  buffer[0] = input;

  // Вычисление ошибки для фильтров обоих каналов
  float error = input - prev_val_dir;
  
  // Для: Перекрестное складывание сигналов
  float output_ch = input - prev_val_inv; // Ex: левый сигнал - опоздавший отфильтрованный правый

  recalc_fir(w_coeffs, error, buffer);

  return output_ch;
}

float apply_filter(float signal, float w_coeffs[FILTER_ORDER]){ // применить фильтр полосовой, отказ от audiotools
  float output_val = 0.0f;
  for (int i=0; i<FILTER_ORDER; i++){
    output_val += signal*w_coeffs[i];
  }
  return output_val;
}


// ---------------- I2S настройка микрофонов и динамика ----------------
void setupI2SMic() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // best for aids
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 32,
    .dma_buf_len = bufer_i2s_lenb,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_MIC_SCK,
    .ws_io_num = I2S_MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SD
  };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

void setupI2SSpeaker() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // due to documentation
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 32,
    .dma_buf_len = bufer_i2s_lenb,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SPK_SCK,
    .ws_io_num = I2S_SPK_WS,
    .data_out_num = I2S_SPK_SD,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_1);
}

size_t record_i2s(int32_t* input, int16_t* output){
  size_t bytes_read = 0;
  i2s_read(I2S_NUM_0, input, sizeof(input), &bytes_read, portMAX_DELAY);

  size_t samples = bytes_read / sizeof(int32_t);
  size_t frames  = samples / 2;
  if (frames > 0){
    Serial.println("frames > 0");
  }

  // stereo per-sample filtering
  for (size_t i = 0; i < frames; ++i) {
    // 1. read l, r, normalize
    size_t li = (i << 1);
    size_t ri = li + 1;
    float xl = normalize(input[li]);
    float xr = normalize(input[ri]);

    // 2. calculate an error from the previous filtering
    float yl = calc_error_make_y(xl, pBufL[i], pBufR[i], w_left); // для левого канала 
    float yr = calc_error_make_y(xr, pBufR[i], pBufL[i], w_right); // для правого канала 
     
    // 3. calculate the result from the previous filtering
    float result = yl+yr;
    output[i] = normalize_speaker(result);

    // 4. filter for the next iteration
    pBufL[i] = apply_filter(yl, w_left);
    pBufR[i] = apply_filter(yr, w_right);
  }
  return bytes_read;
}

