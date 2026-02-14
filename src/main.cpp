#include "dac_spearker.h"
#include "sd_wav.h" // if don't need SD support - comment it

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
#ifdef DAC_SPEARKER_H
  delay(200);
  setupI2SMic();
  setupI2SSpeaker();
  Serial.println("stereo pipeline ready.");
#endif
#ifdef SD_WAV_H
   // Инициализация microSD-карты
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI); // Инициализация SPI с указанными пинами
  if (!SD.begin(SD_CS)) { // Инициализация microSD-карты с указанным пином выбора
    Serial.println("Error while init of microSD!");
    while(1); // Бесконечный цикл при ошибке
  }
  Serial.println("microSD initialized.");

    // Настройка кнопки
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Вход с подтягивающим резистором
  // Настройка прерывания на спад сигнала (нажатие кнопки)
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
  
  Serial.println("Готов к работе. Нажмите кнопку для начала записи.");
#endif

}

void loop() {
#ifdef DAC_SPEAKER_H // Работаем без SD
    #ifndef SD_WAV_H
  size_t frames = record_i2s(input, output);
  size_t bytes_written = 0;
  i2s_write(I2S_NUM_1, output, frames * sizeof(int16_t), &bytes_written, portMAX_DELAY);
    #endif
#endif
#ifdef SD_WAV_H // Filter and SD
  // Обработка нажатия кнопки
  if (buttonPressed) {
    buttonPressed = false; // Сбрасываем флаг
    delay(50); // Задержка для подавления дребезга контактов
    
    // В зависимости от текущего состояния, начинаем или останавливаем запись
    if (!isRecording) {
      startRecording();
    } else {
      stopRecording();
    }
  }
   // Если идет запись, читаем данные с I2S и записываем на microSD-карту
  if (isRecording) {
    size_t bytesRead = record_i2s(input, output);
    
    // Если данные прочитаны, записываем их в файл
    if (bytesRead > 0) {
      uint8_t buffer[BUFFER_FRAMES * 2];
      convertToInt8Buffer(output, buffer);
      audioFile.write((const uint8_t *)buffer, bytesRead);
    }
  }
#endif
}