#include <driver/i2s.h>
#include <SPIFFS.h>

#define I2S_WS 5
#define I2S_SD 18
#define I2S_SCK 4
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE   (8000)
#define I2S_SAMPLE_BITS   (16)
#define I2S_READ_LEN      (32) // For 128 bytes of total buffer size
#define RECORD_TIME       (20) //Seconds
#define I2S_CHANNEL_NUM   (2)
#define FLASH_RECORD_SIZE (I2S_READ_LEN*2)//(I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * (I2S_SAMPLE_BITS / 8) * RECORD_TIME)

File file;
const char filename[] = "/recording.wav";
const int headerSize = 44;

void wavHeader(byte* header, int wavSize){
  header[0] = 'R'; //
  header[1] = 'I'; //
  header[2] = 'F'; //
  header[3] = 'F'; //
  unsigned int fileSize = wavSize + headerSize - 8; //
  Serial.printf("fileSize = %d\n", fileSize);
  header[4] = (byte)(fileSize & 0xFF); //
  header[5] = (byte)((fileSize >> 8) & 0xFF); //
  header[6] = (byte)((fileSize >> 16) & 0xFF); // 
  header[7] = (byte)((fileSize >> 24) & 0xFF); //
  header[8] = 'W'; //
  header[9] = 'A'; //
  header[10] = 'V'; //
  header[11] = 'E'; //
  header[12] = 'f'; //
  header[13] = 'm'; //
  header[14] = 't'; //
  header[15] = ' '; //
  header[16] = 0x10; //
  header[17] = 0x00; //
  header[18] = 0x00; //
  header[19] = 0x00; //
  header[20] = 0x01; //
  header[21] = 0x00; //
  
  header[22] = 0x02; // mono - stereo
  header[23] = 0x00; // mono - stereo

  header[24] = 0x80; // SampleRate
  header[25] = 0x3E; // SampleRate - поменял
  header[26] = 0x00; // SampleRate
  header[27] = 0x00; // SampleRate

  header[28] = 0x00; //byteRate - SampleRate * BitsPerSample * Channels / 8
  header[29] = 0xFA; //byteRate - поменял
  header[30] = 0x00; //byteRate
  header[31] = 0x00; //byteRate

  header[32] = 0x04; //blockAlign - BitsPerSample * Channels / 8
  header[33] = 0x00; //blockAlign

  header[34] = 0x10; //bitsPerSample
  header[35] = 0x00; //bitsPerSample

  header[36] = 'd'; //
  header[37] = 'a'; //
  header[38] = 't'; //
  header[39] = 'a'; //
  header[40] = (byte)(wavSize & 0xFF);
  header[41] = (byte)((wavSize >> 8) & 0xFF);
  header[42] = (byte)((wavSize >> 16) & 0xFF);
  header[43] = (byte)((wavSize >> 24) & 0xFF);
  
}


void listSPIFFS(void) {
  Serial.println(F("\r\nListing SPIFFS files:"));
  static const char line[] PROGMEM =  "=================================================";

  Serial.println(FPSTR(line));
  Serial.println(F("  File name                              Size"));
  Serial.println(FPSTR(line));

  fs::File root = SPIFFS.open("/");
  if (!root) {
    Serial.println(F("Failed to open directory"));
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(F("Not a directory"));
    return;
  }

  fs::File file = root.openNextFile();
  while (file) {

    if (file.isDirectory()) {
      Serial.print("DIR : ");
      String fileName = file.name();
      Serial.print(fileName);
    } else {
      String fileName = file.name();
      Serial.print("  " + fileName);
      // File path can be 31 characters maximum in SPIFFS
      int spaces = 33 - fileName.length(); // Tabulate nicely
      if (spaces < 1) spaces = 1;
      while (spaces--) Serial.print(" ");
      String fileSize = (String) file.size();
      spaces = 10 - fileSize.length(); // Tabulate nicely
      if (spaces < 1) spaces = 1;
      while (spaces--) Serial.print(" ");
      Serial.println(fileSize + " bytes");
    }

    file = root.openNextFile();
  }

  Serial.println(FPSTR(line));
  Serial.println("Finished listing");
  delay(1000);
}

void deleteAllSPIFFSFiles() {
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }
  
  Serial.println("SPIFFS mounted successfully");
  Serial.println("Deleting all files from SPIFFS...");
  
  // Open root directory
  File root = SPIFFS.open("/");
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }
  
  File file = root.openNextFile();
  int fileCount = 0;
  
  // Iterate through all files
  while (file) {
    if (!file.isDirectory()) {
      String filePath = String(file.path());
      Serial.print("Deleting file: ");
      Serial.println(filePath);
      
      // Delete the file
      if (SPIFFS.remove(filePath)) {
        Serial.println("File deleted successfully");
        fileCount++;
      } else {
        Serial.println("Error deleting file");
      }
    }
    file = root.openNextFile();
  }
  
  Serial.print("Total files deleted: ");
  Serial.println(fileCount);
  
  // Verify that no files remain
  Serial.println("Verifying all files are deleted...");
  root = SPIFFS.open("/");
  file = root.openNextFile();
  int remainingFiles = 0;
  
  while (file) {
    if (!file.isDirectory()) {
      remainingFiles++;
      Serial.print("Remaining file: ");
      Serial.println(file.path());
    }
    file = root.openNextFile();
  }
  
  if (remainingFiles == 0) {
    Serial.println("All files successfully deleted!");
  } else {
    Serial.print(remainingFiles);
    Serial.println(" files remain");
  }
  
  SPIFFS.end();
}

void SPIFFSInit(){
  if(!SPIFFS.begin(true)){
    Serial.println("SPIFFS initialisation failed!");
    while(1) yield();
  }
  Serial.print("SPIFFS total bytes: ");Serial.println(SPIFFS.totalBytes()); // TODO :  выяснить почему вся ФС занята а файлы не выводятся. И еще зацикливание происходит
  Serial.print("SPIFFS used bytes: ");Serial.println(SPIFFS.usedBytes());

  SPIFFS.remove(filename);
  file = SPIFFS.open(filename, FILE_WRITE);
  if(!file){
    Serial.println("File is not available!");
  }

  byte header[headerSize];
  wavHeader(header, FLASH_RECORD_SIZE);

  file.write(header, headerSize);

  file.flush();
  

  listSPIFFS();
}

void i2sInit(){
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 32,
    .dma_buf_len = 512,
    .use_apll = 1
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);


  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}


void i2s_adc_data_scale(uint8_t *d_buff, int16_t *s_buff, uint32_t len)
{
    uint32_t j = 0;

    
    uint32_t samples = len ;

    for (uint32_t i = 0; i < samples; i++) {
        // Берём 16-битный signed sample
        int16_t sample = s_buff[i];

        // Приводим к unsigned диапазону (0..65535)
        uint16_t dac_value = (uint16_t)(sample + 32768);

        // Старший байт ноль (как было раньше)
        d_buff[j++] = 0;

        // Масштабирование под 8 бит (аналог твоего *256/2048)
        d_buff[j++] = dac_value >> 8;
    }
}


void i2s_adc(void *arg)
{
  int samples = I2S_READ_LEN;
  int flash_wr_size = 0;
  size_t bytes_read;

  int16_t* i2s_read_buff = (int16_t*) calloc(samples*2, sizeof(int16_t));
  int16_t* i2s_filtered_samples = (int16_t*) calloc(samples, sizeof(int16_t));
  uint8_t* flash_write_buff = (uint8_t*) calloc(samples, sizeof(uint8_t));
  
  Serial.println(" *** Recording Start *** ");
  while (flash_wr_size < FLASH_RECORD_SIZE) {
    //read data from I2S bus, in this case, from ADC.
    i2s_read(I2S_PORT, (void*) i2s_read_buff, samples*2, &bytes_read, portMAX_DELAY);

    // Filtering
    for (size_t i = 0; i < samples; ++i) {
      size_t li = (i << 1);
      size_t ri = li + 1;
      i2s_filtered_samples[i] = i2s_read_buff[li] - i2s_read_buff[ri];
    }

    //save original data from I2S(ADC) into flash.
    i2s_adc_data_scale(flash_write_buff, i2s_filtered_samples, samples);
    file.write((const byte*) flash_write_buff, samples);     
     file.flush();
    flash_wr_size += samples*sizeof(int16_t);

    Serial.printf("flash_wr_size % = %.2f\n", (flash_wr_size * 1.0)/(FLASH_RECORD_SIZE * 1.0));
  }
  Serial.println(" *** Recording Finished *** ");
  file.close();

  free(i2s_read_buff);
  i2s_read_buff = NULL;
  free(flash_write_buff);
  flash_write_buff = NULL;
  
  listSPIFFS();
  vTaskDelete(NULL);
}

