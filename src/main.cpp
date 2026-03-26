#include <Arduino.h>
#include "driver/i2s.h"
#include <string.h>

// ─── Аудио параметры ───
#define SAMPLE_RATE    48000
#define DMA_BUF_COUNT  4
#define DMA_BUF_LEN    64
#define GAIN           4.0f

// ─── Режим записи через Serial ───
#define SERIAL_BAUD    921600
#define REC_DOWNSAMPLE 3                // no downsampling → full 48000 Hz
#define REC_SAMPLE_RATE (SAMPLE_RATE / REC_DOWNSAMPLE)  // 48000

static bool recording = false;

// ─── Пины ───
#define I2S_MIC_WS   GPIO_NUM_5
#define I2S_MIC_SCK  GPIO_NUM_4
#define I2S_MIC_SD   GPIO_NUM_18

#define I2S_SPK_WS   GPIO_NUM_21
#define I2S_SPK_SCK  GPIO_NUM_23
#define I2S_SPK_SD   GPIO_NUM_22

// ─── Утилиты ───
static inline int16_t clamp16(float y) {
  if (y > 32767.0f) return 32767;
  if (y < -32768.0f) return -32768;
  return (int16_t)lrintf(y);
}

static inline float normalize(int32_t value) {
  return (float)(value >> 8) / 8388608.0f;
}

// ─── I2S setup ───
void setupI2SMic() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = true,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_MIC_SCK,
    .ws_io_num = I2S_MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SD
  };
  esp_err_t err = i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  Serial.printf("[MIC] install: %s\n", esp_err_to_name(err));
  err = i2s_set_pin(I2S_NUM_0, &pins);
  Serial.printf("[MIC] set_pin: %s\n", esp_err_to_name(err));
  i2s_zero_dma_buffer(I2S_NUM_0);
}

void setupI2SSpeaker() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_SPK_SCK,
    .ws_io_num = I2S_SPK_WS,
    .data_out_num = I2S_SPK_SD,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  esp_err_t err = i2s_driver_install(I2S_NUM_1, &cfg, 0, NULL);
  Serial.printf("[SPK] install: %s\n", esp_err_to_name(err));
  err = i2s_set_pin(I2S_NUM_1, &pins);
  Serial.printf("[SPK] set_pin: %s\n", esp_err_to_name(err));
  i2s_zero_dma_buffer(I2S_NUM_1);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  setupI2SMic();
  setupI2SSpeaker();
  Serial.println("═══════════════════════════════════════");
  Serial.println("  Прямой аудиомонитор (без фильтрации)");
  Serial.printf("  SR: %d Hz | Gain: %.1f\n", SAMPLE_RATE, GAIN);
  Serial.printf("  Serial: %d baud | Rec SR: %d Hz\n", SERIAL_BAUD, REC_SAMPLE_RATE);
  Serial.println("  Команды: 'R' = запись, 'S' = стоп");
  Serial.println("═══════════════════════════════════════");
}

void loop() {
  int32_t input[64 * 2];  // стерео вход (64 сэмпла * 2 канала)
  int16_t output[64];     // моно выход

  size_t bytes_read = 0;
  i2s_read(I2S_NUM_0, input, sizeof(input), &bytes_read, portMAX_DELAY);
  size_t frames = bytes_read / sizeof(int32_t) / 2;

  // Преобразование стерео 32-бит в моно 16-бит с усилением
  for (size_t i = 0; i < frames; ++i) {
    // Конвертация 32-бит I2S в float и микширование стерео в моно
    float left = normalize(input[i * 2]);
    float right = normalize(input[i * 2 + 1]);
    float mono = (left + right) * 0.5f * GAIN;
    
    // Преобразование обратно в int16
    output[i] = clamp16(mono * 32767.0f);
  }

  // Отправка в динамик
  size_t bytes_written = 0;
  i2s_write(I2S_NUM_1, output, frames * sizeof(int16_t), &bytes_written, portMAX_DELAY);

  // ─── Serial: команды и стриминг ───
  while (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'R' && !recording) {
      recording = true;
      uint8_t marker[] = {0xAA, 0x55};  // маркер начала
      Serial.write(marker, 2);
    } else if (cmd == 'S' && recording) {
      recording = false;
      uint8_t marker[] = {0x55, 0xAA};  // маркер конца
      Serial.write(marker, 2);
    }
  }

  if (recording) {
    // Даунсэмпл: каждый REC_DOWNSAMPLE-й сэмпл
    for (size_t i = 0; i < frames; i += REC_DOWNSAMPLE) {
      Serial.write((uint8_t*)&output[i], sizeof(int16_t));
    }
  }
}