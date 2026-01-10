// #include "wav_record_kirill.h"

// void setup() {
//   deleteAllSPIFFSFiles();
//   // put your setup code here, to run once:
//   Serial.begin(115200);
//   SPIFFSInit();
//   i2sInit();
//   Serial.println("I2S init finished");
//   xTaskCreate(i2s_adc, "i2s_adc", 1024 * 16, NULL, 1, NULL); // 1024 --> 256
// }

// void loop() {
//   // put your main code here, to run repeatedly:

// }

#include "dac_spearker.h"

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  setupI2S();

  Serial.print("FIR taps: "); Serial.println(FIR_TAPS);
  Serial.println("i2s_std + AudioTools::FIR stereo pipeline ready.");
}

//static bool printed = false;

void loop() {
  static int16_t inBuf[BUFFER_FRAMES * 2];
  static int16_t outBuf[BUFFER_FRAMES * 2];

  size_t bytes_read = 0;
  if (i2s_channel_read(rx_handle, inBuf, sizeof(inBuf), &bytes_read, 1000) != ESP_OK || bytes_read == 0) return;

  size_t samples = bytes_read / sizeof(int16_t);
  size_t frames  = samples / 2;

  // stereo per-sample filtering
  for (size_t i = 0; i < frames; ++i) {
    size_t li = (i << 1);
    size_t ri = li + 1;

    float xl = (float)inBuf[li];
    float xr = (float)inBuf[ri];

    float yl = firL.process(xl)* GAIN;
    float yr = firR.process(xr)* GAIN;
    int y_result = clamp16(yl) - clamp16(yr);
    outBuf[i] = y_result;
    // outBuf[li] = clamp16(yl);
    // outBuf[ri] = clamp16(yr);
  }

  size_t bytes_written = 0;
  i2s_channel_write(tx_handle, outBuf, frames *2* sizeof(int16_t), &bytes_written, 1000);
}
