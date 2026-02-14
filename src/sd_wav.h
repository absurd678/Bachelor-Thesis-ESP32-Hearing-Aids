#include <Arduino.h>          // Основная библиотека Arduino
#include <SD.h>               // Библиотека для работы с microSD-картой
#include <SPI.h>              // Библиотека для работы с интерфейсом SPI

#ifndef SD_WAV_H
#define SD_WAV_H
#endif

#define SAMPLE_RATE      44100
// Настройки microSD-карты
#define SD_CS 5              // Пин для выбора microSD-карты (Chip Select)
#define SPI_MOSI 23          // Пин MOSI (Master Out Slave In) для SPI
#define SPI_MISO 19          // Пин MISO (Master In Slave Out) для SPI
#define SPI_SCK 18           // Пин SCK (Serial Clock) для SPI

// Настройки кнопки
#define BUTTON_PIN 0         // Встроенная кнопка на NodeMCU-32S 38pin (расположена справа от USB-разъёма)

File audioFile;              // Объект для работы с файлом на microSD-карте
volatile bool isRecording = false; // Флаг состояния записи (volatile для работы в прерываниях)
volatile bool buttonPressed = false; // Флаг нажатия кнопки (volatile для работы в прерываниях)
int fileNumber = 0;          // Счетчик для нумерации файлов

// Функция обработки прерывания кнопки
void IRAM_ATTR buttonISR() {
  buttonPressed = true;      // Устанавливаем флаг при нажатии кнопки
  // IRAM_ATTR указывает, что функция должна быть размещена в RAM (а не во flash) для быстрого выполнения
}

// Функция записи заголовка WAV файла
void writeWavHeader(File file, int sampleRate, uint32_t totalSamples) {
  // Рассчитываем необходимые параметры для заголовка
  uint32_t byteRate = sampleRate * 2;  // 16-bit = 2 bytes per sample
  uint32_t totalDataSize = totalSamples * 2; // Общий размер данных в байтах
  uint32_t totalSize = totalDataSize + 36; // Общий размер файла (данные + заголовок)
  
  // RIFF header (основной заголовок файла)
  file.write('R'); file.write('I'); file.write('F'); file.write('F'); // "RIFF" метка
  // Размер файла (минус 8 байт для "RIFF" и размера)
  file.write((uint8_t)(totalSize & 0xFF));         // Младший байт
  file.write((uint8_t)((totalSize >> 8) & 0xFF));  // Следующий байт
  file.write((uint8_t)((totalSize >> 16) & 0xFF)); // ...
  file.write((uint8_t)((totalSize >> 24) & 0xFF)); // Старший байт
  file.write('W'); file.write('A'); file.write('V'); file.write('E'); // "WAVE" метка
  
  // fmt subchunk (описание формата данных)
  file.write('f'); file.write('m'); file.write('t'); file.write(' '); // "fmt " метка
  file.write(16); file.write(0); file.write(0); file.write(0);  // Subchunk1Size = 16 (для PCM)
  file.write(1); file.write(0);  // AudioFormat = PCM (линейный несжатый)
  file.write(1); file.write(0);  // NumChannels = 1 (моно)
  // Частота дискретизации
  file.write((uint8_t)(sampleRate & 0xFF));
  file.write((uint8_t)((sampleRate >> 8) & 0xFF));
  file.write((uint8_t)((sampleRate >> 16) & 0xFF));
  file.write((uint8_t)((sampleRate >> 24) & 0xFF));
  // Байтрейт (байт в секунду)
  file.write((uint8_t)(byteRate & 0xFF));
  file.write((uint8_t)((byteRate >> 8) & 0xFF));
  file.write((uint8_t)((byteRate >> 16) & 0xFF));
  file.write((uint8_t)((byteRate >> 24) & 0xFF));
  file.write(2); file.write(0);  // BlockAlign = 2 (байт на сэмпл для моно 16-бит)
  file.write(16); file.write(0);  // BitsPerSample = 16 (16-битное аудио)
  
  // data subchunk (начало блока данных)
  file.write('d'); file.write('a'); file.write('t'); file.write('a'); // "data" метка
  // Размер данных в байтах
  file.write((uint8_t)(totalDataSize & 0xFF));
  file.write((uint8_t)((totalDataSize >> 8) & 0xFF));
  file.write((uint8_t)((totalDataSize >> 16) & 0xFF));
  file.write((uint8_t)((totalDataSize >> 24) & 0xFF));
}

// Функция начала записи
void startRecording() {
  // Создание нового файла с уникальным именем
  char filename[20];
  do {
    sprintf(filename, "/recording%d.wav", fileNumber++); // Формируем имя файла
  } while(SD.exists(filename)); // Проверяем, не существует ли уже файл с таким именем
  
  // Открываем файл для записи
  audioFile = SD.open(filename, FILE_WRITE);
  if(!audioFile) {
    Serial.println("Ошибка создания файла!");
    return;
  }
  
  // Записываем временный заголовок WAV файла (с нулевым размером данных)
  // Фактический размер данных будет обновлен при остановке записи
  writeWavHeader(audioFile, SAMPLE_RATE, 0);
  
  // Устанавливаем флаг записи
  isRecording = true;
  Serial.println("Начало записи...");
}

// Функция остановки записи
void stopRecording() {
  // Сбрасываем флаг записи
  isRecording = false;
  
  if (audioFile) {
    // Вычисляем количество записанных сэмплов
    // Размер файла минус размер заголовка (36 байт), деленный на 2 (16 бит = 2 байта на сэмпл)
    uint32_t totalSamples = (audioFile.size() - 36) / 2;
    
    // Переходим к началу файла и обновляем заголовок с правильным размером данных
    audioFile.seek(0);
    writeWavHeader(audioFile, SAMPLE_RATE, totalSamples);
    
    // Закрываем файл
    audioFile.close();
    Serial.println("Запись завершена.");
    Serial.print("Записано сэмплов: ");
    Serial.println(totalSamples);
  }
}

void convertToInt8Buffer(const int16_t input[32], uint8_t output[64]) { // when use with filters
    for (int i = 0; i < 32; ++i) {
        int16_t value = input[i];
        // Младший байт (low byte) — записываем в чётный индекс
        output[2 * i] = static_cast<uint8_t>(value & 0xFF);
        // Старший байт (high byte) — записываем в нечётный индекс
        output[2 * i + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    }
}
