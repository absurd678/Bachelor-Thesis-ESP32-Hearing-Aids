#include "wav_record.h"


void setup() {
  deleteAllSPIFFSFiles();
  // put your setup code here, to run once:
  Serial.begin(115200);
  SPIFFSInit();
#ifdef DAC_SPEAKER
  setupI2SMic();
  setupI2SSpeaker();
#else
  i2sInit();
#endif
  Serial.println("I2S init finished");
  xTaskCreate(i2s_adc, "i2s_adc", 1024 * 16, NULL, 1, NULL); // 1024 --> 256
}

void loop() {
  // put your main code here, to run repeatedly:

}