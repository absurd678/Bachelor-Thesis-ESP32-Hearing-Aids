"""
Запись аудио с ESP32 Hearing Aid через Serial.

Использование:
    python record.py COM3              # запись в WAV/output.wav и PCM/output.pcm
    python record.py COM3 my_rec       # запись в WAV/my_rec.wav и PCM/my_rec.pcm
    python record.py /dev/ttyUSB0      # Linux

Скрипт подключается к ESP32, отправляет команду 'R' (старт записи),
принимает бинарные PCM-данные (int16, mono, 14700 Hz),
и по Ctrl+C или команде 'S' сохраняет в WAV и сырой PCM.
"""

import sys
import time
import wave
import os
import serial

BAUD_RATE = 1000000
SAMPLE_RATE = 48000  # 44100 / 3
CHANNELS = 1
SAMPLE_WIDTH = 2  # int16 = 2 bytes

MARKER_START = b'\xaa\x55'
MARKER_STOP  = b'\x55\xaa'

DIR_WAV = "WAV"
DIR_PCM = "PCM"


def main():
    if len(sys.argv) < 2:
        print("Использование: python record.py <COM-порт> [имя_файла_без_расширения]")
        print("Пример:        python record.py COM3")
        print("               python record.py COM3 my_rec")
        sys.exit(1)

    port = sys.argv[1]
    base_name = sys.argv[2] if len(sys.argv) > 2 else "output"

    # Создаём папки, если не существуют
    os.makedirs(DIR_WAV, exist_ok=True)
    os.makedirs(DIR_PCM, exist_ok=True)

    wav_file = os.path.join(DIR_WAV, f"{base_name}.wav")
    pcm_file = os.path.join(DIR_PCM, f"{base_name}.pcm")

    print(f"Подключение к {port} @ {BAUD_RATE} baud...")
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    time.sleep(2)  # ждём загрузку ESP32

    # Очищаем буфер (стартовые сообщения ESP32)
    ser.reset_input_buffer()

    print("Отправляю команду записи 'R'...")
    ser.write(b'R')

    # Ждём маркер начала
    print("Жду маркер начала (0xAA 0x55)...")
    buf = b''
    timeout_start = time.time()
    while time.time() - timeout_start < 5:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            idx = buf.find(MARKER_START)
            if idx >= 0:
                buf = buf[idx + 2:]  # отбрасываем всё до маркера
                break
    else:
        print("Таймаут: маркер начала не получен")
        ser.close()
        sys.exit(1)

    print("Запись началась! Нажмите Ctrl+C для остановки.")
    print(f"  WAV → {wav_file}")
    print(f"  PCM → {pcm_file}")

    audio_data = bytearray(buf)  # данные после маркера
    samples = len(buf) // SAMPLE_WIDTH

    try:
        while True:
            if ser.in_waiting:
                chunk = ser.read(ser.in_waiting)

                # Проверяем маркер конца в полученных данных
                stop_idx = chunk.find(MARKER_STOP)
                if stop_idx >= 0:
                    audio_data.extend(chunk[:stop_idx])
                    samples += stop_idx // SAMPLE_WIDTH
                    print("\nПолучен маркер конца от ESP32.")
                    break

                audio_data.extend(chunk)
                samples += len(chunk) // SAMPLE_WIDTH

                # Прогресс
                duration = samples / SAMPLE_RATE
                sys.stdout.write(f"\r  Записано: {duration:.1f} сек ({samples} сэмплов, {len(audio_data)} байт)")
                sys.stdout.flush()
            else:
                time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n\nОстановка по Ctrl+C...")
        ser.write(b'S')
        time.sleep(0.1)
        # Дочитываем остатки
        if ser.in_waiting:
            tail = ser.read(ser.in_waiting)
            stop_idx = tail.find(MARKER_STOP)
            if stop_idx >= 0:
                audio_data.extend(tail[:stop_idx])
            else:
                audio_data.extend(tail)

    ser.close()

    # Выравниваем до целого числа сэмплов
    trim = len(audio_data) % SAMPLE_WIDTH
    if trim:
        audio_data = audio_data[:-trim]

    total_samples = len(audio_data) // SAMPLE_WIDTH
    duration = total_samples / SAMPLE_RATE

    if total_samples == 0:
        print("Нет данных для сохранения.")
        sys.exit(1)

    # Сохраняем WAV
    with wave.open(wav_file, 'w') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(SAMPLE_WIDTH)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(bytes(audio_data))

    # Сохраняем сырой PCM
    with open(pcm_file, 'wb') as f:
        f.write(bytes(audio_data))

    print(f"\nФайлы сохранены:")
    print(f"  {wav_file}  (PCM 16-bit WAV)")
    print(f"  {pcm_file}  (Raw PCM, signed 16-bit little-endian)")
    print(f"\n  Sample rate:  {SAMPLE_RATE} Hz")
    print(f"  Длительность: {duration:.1f} сек")
    print(f"  Размер:       {len(audio_data)} байт")
    print(f"\nВоспроизведение PCM через ffplay:")
    print(f"  ffplay -f s16le -ar {SAMPLE_RATE} -ac {CHANNELS} {pcm_file}")


if __name__ == "__main__":
    main()