#pragma once
// ---------- I2S ----------
#define SAMPLE_RATE 48000
#define I2S_MIC_WS GPIO_NUM_5
#define I2S_MIC_SCK GPIO_NUM_4
#define I2S_MIC_SD GPIO_NUM_18
#define I2S_SPK_WS GPIO_NUM_21
#define I2S_SPK_SCK GPIO_NUM_23
#define I2S_SPK_SD GPIO_NUM_22
#define DMA_BUF_COUNT 8
#define SERIAL_BAUD 921600
// -------- LMS ---------
#define TAPS 64
#define MU 0.05f
#define DELAY_SAMPLES 1 // now unused. handy when using deque or fifo
// --------- FFT ----------
#define FFT_SIZE 512
#define TWO_PI_F 6.28318530717958647692f
#define AUDIO_BUF_LEN 256
// --------- RNN ------------
#define kTensorArenaSize (64 * 1024)
// #define TFLITE_EMULATE_FLOAT
