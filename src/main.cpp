#include "wav_record_kirill.h"


void setup() {
  deleteAllSPIFFSFiles();
  // put your setup code here, to run once:
  Serial.begin(115200);
  SPIFFSInit();
  i2sInit();
  xTaskCreate(i2s_adc, "i2s_adc", 1024 * 2*2, NULL, 1, NULL); // 1024 --> 256
}

void loop() {
  // put your main code here, to run repeatedly:

}