#include <driver/i2s.h>
#include <SPIFFS.h>

#define I2S_WS 5
#define I2S_SD 18
#define I2S_SCK 4
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE   (16000)
#define I2S_SAMPLE_BITS   (16)
#define I2S_READ_LEN      (16 * 1024) // Read buffer length
#define RECORD_TIME       (20) //Seconds
#define I2S_CHANNEL_NUM   (2)
#define FLASH_RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * (I2S_SAMPLE_BITS / 8) * RECORD_TIME)

File file;
const char filename[] = "/recording.wav";
const int headerSize = 44;

void wavHeader(byte* header, int wavSize){
  header[0] = 'R'; //
  header[1] = 'I'; //
  header[2] = 'F'; //
  header[3] = 'F'; //
  unsigned int fileSize = wavSize + headerSize - 8; //
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
  Serial.println();
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

  SPIFFS.remove(filename);
  file = SPIFFS.open(filename, FILE_WRITE);
  if(!file){
    Serial.println("File is not available!");
  }

  byte header[headerSize];
  wavHeader(header, FLASH_RECORD_SIZE);

  file.write(header, headerSize);
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


void i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len)
{
    uint32_t j = 0;
    uint32_t dac_value = 0;
    for (int i = 0; i < len; i += 2) {
        dac_value = ((((uint16_t) (s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
        d_buff[j++] = 0;
        d_buff[j++] = dac_value * 256 / 2048; // 256 ---> 1024
    }
}

float phi = 5.7;
  int A = 32767;
  float pi = 3.141592653589;
void i2s_adc(void *arg)
{
    int i2s_read_len = I2S_READ_LEN;
    int flash_wr_size = 0;
    size_t bytes_read;

    char* i2s_read_buff = (char*) calloc(i2s_read_len, sizeof(char));
    uint8_t* flash_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));

    // i2s_read(I2S_PORT, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
    // i2s_read(I2S_PORT, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
    
    Serial.println(" *** Recording Start *** ");
    while (flash_wr_size < FLASH_RECORD_SIZE) {
        // //read data from I2S bus, in this case, from ADC.
        // i2s_read(I2S_PORT, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);

        // //save original data from I2S(ADC) into flash.
        // i2s_adc_data_scale(flash_write_buff, (uint8_t*)i2s_read_buff, i2s_read_len);
        // file.write((const byte*) flash_write_buff, i2s_read_len);

        for (int frame=0; frame<I2S_READ_LEN/2; frame++){ //  TODO: забить хуй и вставить фильтр
          float xl = A*sinf(2*pi*frame/5);
          float xr = A*sinf(2*pi*frame/5 + phi);
          i2s_read_buff[frame<<1] = (short)xl;
          i2s_read_buff[(frame<<1) + 1] = (short)xr;
        }
        //save original data from I2S(ADC) into flash.
        i2s_adc_data_scale(flash_write_buff, (uint8_t*)i2s_read_buff, i2s_read_len);
        file.write((const byte*) flash_write_buff, i2s_read_len);
        

        flash_wr_size += i2s_read_len;

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

