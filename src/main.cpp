
#include <Arduino.h>
#include "driver/i2s.h"
#include "driver/gpio.h"

// Audio parameters
#define SAMPLE_RATE      44100
#define BUFFER_FRAMES    32    // number of L/R frames per read/write (adjust as needed)
#define MU 0.00005f // множитель сходимости для LMS фильтра
//пины для INMP441
#define I2S_MIC_WS GPIO_NUM_15
#define I2S_MIC_SCK GPIO_NUM_14
#define I2S_MIC_SD GPIO_NUM_32
//пины для MAX98357A
#define I2S_SPK_WS GPIO_NUM_26
#define I2S_SPK_SCK GPIO_NUM_27
#define I2S_SPK_SD GPIO_NUM_25
#define bufer_i2s_lenb 512
 // LRC - D5 -- соответствие пинов для справки
// BCLK - D4
// DIN - D18
// GAIN - GND 
// Vin - 5V

#define DEBUG_PLOT 0   // set 0 to disable
static const char* TAG = "I2S_AUDIO";

// ---------------- FIR coefficients (stereo band-pass ~300..3400 Hz, example) ----------------
float w_left[] = { // кэфы для левого киха
  0.1200, 0.1400, 0.1900, 0.2600, 0.1900, 0.1400, 0.1200
};

float w_right[] = { // кэфы для правого киха
  0.1200, 0.1400, 0.1900, 0.2600, 0.1900, 0.1400, 0.1200
};

static const size_t FILTER_ORDER = sizeof(w_left)/sizeof(w_left[0]);
const float GAIN = 1.0f; // потом можно менять 1...10
float var_gain = GAIN;
uint32_t counter = 0; // Итеративное изменение GAIN каждые 10 секунд
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


float calc_error_make_y(float input, float prev_val_dir, float prev_val_inv, float w_coeffs[FILTER_ORDER]){ // // вх сигнал, прошлый этого же канала, прошлый из противоположного канала для адаптивного фильтра

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
  float output = 0.0f;
  for (int i=0; i<FILTER_ORDER; i++){
    output += signal*w_coeffs[i];
  }
  return output;
}


// ---------------- I2S настройка микрофонов и динамика ----------------
void setupI2SMic() {
  esp_err_t err;
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // best for aids
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 32,
    .dma_buf_len = bufer_i2s_lenb,
    .use_apll = true,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_MIC_SCK,
    .ws_io_num = I2S_MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SD
  };
  err  = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  ESP_ERROR_CHECK(err); 
  err  = i2s_set_pin(I2S_NUM_0, &pin_config);
  ESP_ERROR_CHECK(err);
  err  = i2s_zero_dma_buffer(I2S_NUM_0);
  ESP_ERROR_CHECK(err);
}

void setupI2SSpeaker() {
  esp_err_t err;
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // due to documentation
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
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
  err  = i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL);
  ESP_ERROR_CHECK(err); 
  err  = i2s_set_pin(I2S_NUM_1, &pin_config);
  ESP_ERROR_CHECK(err);
  err  = i2s_zero_dma_buffer(I2S_NUM_1);
  ESP_ERROR_CHECK(err);
}
// ---------------- Arduino ----------------
void setup() {
  Serial.begin(921600);
  delay(200);
  setupI2SMic();
  setupI2SSpeaker();
  Serial.println("stereo pipeline ready.");
}

void loop() {

  int32_t input[BUFFER_FRAMES * 2]; // mic in
  int16_t output[BUFFER_FRAMES]; // speaker out
  float pBufL[BUFFER_FRAMES] = {0}; // filtered signal saved
  float pBufR[BUFFER_FRAMES] = {0}; // filtered signal saved
  float error_array_left_fir[FILTER_ORDER]; // left filter error
  float error_array_right_fir[FILTER_ORDER]; // right filter error

  size_t bytes_read = 0;
  i2s_read(I2S_NUM_0, input, sizeof(input), &bytes_read, portMAX_DELAY);
#if DEBUG_PLOT
  if (bytes_read != sizeof(input)) {
    ESP_LOGW(TAG, "Short read: got %d expected %d", bytes_read, sizeof(input));
}
#endif

  size_t samples = bytes_read / sizeof(int32_t);
  size_t frames  = samples / 2;

  //ESP_LOGI(TAG, "frames read: %d, bytes: %d", frames, bytes_read);

  // stereo per-sample filtering
  for (size_t i = 0; i < frames; ++i) {
    // 1. read l, r, normalize
    size_t li = (i << 1);
    size_t ri = li + 1;
    float xl = normalize(input[li]);
    float xr = normalize(input[ri]);

    #if DEBUG_PLOT
      static int plot_counter = 0;
      if (++plot_counter % 4 == 0) {  // print every 4th frame to reduce load
        Serial.print(">mic_left:");
        Serial.println(xl, 4);

        Serial.print(">mic_right:");
        Serial.println(xr, 4);
    }
    #endif

    // 2. calculate an error from the previous filtering
    float yl = calc_error_make_y(xl, pBufL[i], pBufR[i], w_left); // для левого канала 
    float yr = calc_error_make_y(xr, pBufR[i], pBufL[i], w_right); // для правого канала 
    
    // 3. calculate the result from the previous filtering
    float result = yl;//+yr;

    // [DEBUG] NOISY PLACE
    output[i] = normalize_speaker(result)*GAIN; // total result filtered for speaker!
    
    
    // 4. filter for the next iteration
    pBufL[i] = apply_filter(yl, w_left);
    pBufR[i] = apply_filter(yr, w_right);

    #if DEBUG_PLOT
      static int plot_counter1 = 0;
      if (++plot_counter1 % 4 == 0) {  // print every 4th frame to reduce load
        Serial.print(">pBufL:");
        Serial.println(pBufL[i], 4);

        Serial.print(">pBufR:");
        Serial.println(pBufR[i], 4);
    }
    #endif 
  }
  
  size_t bytes_written = 0;
  i2s_write(I2S_NUM_1, output, frames * sizeof(int16_t), &bytes_written, portMAX_DELAY);
  
}
