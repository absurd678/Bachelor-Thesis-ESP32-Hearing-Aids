#include <Arduino.h>
#include <driver/i2s.h>

//--------------------------INMP441-------------------------------------------------
#define I2S_WS 25
#define I2S_SD 32
#define I2S_SCK 33
#define I2S_PORT_MIC I2S_NUM_0
#define I2S_SAMPLE_RATE   (8000)  // Частота, Гц
#define I2S_SAMPLE_BITS   (16)  // Бит в семпле
#define I2S_READ_LEN      (16 * 1024) // Read buffer length

//------------------------MAX98357A----------------------------------------------------
#define LRC 13
#define DIN 14
#define BCLK 12    
#define I2S_PORT_SPEAKER I2S_NUM_1

//------------------------Глобальные переменные----------------------------------------------------
char* i2sBuff = (char*) calloc(I2S_READ_LEN, sizeof(char)); // Буфер семплов
uint32_t DataIdx=0;                           // Индекс смещения по буферу

//-------------------------Объявление функций-----------------------------------
void i2sInit();
void task_micro(void *arg);

// //------------------------Конфиги----------------------------------------------------
//------------------------MAX98357A----------------------------------------------------
static const i2s_config_t i2s_config_speaker = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = I2S_SAMPLE_RATE,                         
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,      
    .dma_buf_count = 4,                             // 4 буфера ДМА
    .dma_buf_len = 128,//64,                            // Проверял dma_buf_len = 64, 128, 1024
    .use_apll=0,
    .tx_desc_auto_clear= true, 
    .fixed_mclk=-1    
};

static const i2s_pin_config_t pin_config_speaker = {
    .mck_io_num = -1,
    .bck_io_num = BCLK,                                 // The bit clock connectiom, goes to pin 27 of ESP32
    .ws_io_num = LRC,                                  // Word select, also known as word select or left right clock
    .data_out_num = DIN,                               // Data out from the ESP32, connect to DIN on 38357A
    .data_in_num = I2S_PIN_NO_CHANGE                  // we are not interested in I2S data into the ESP32
    
};

//--------------------------МИКРОФОН-------------------------------------------------
i2s_config_t i2s_config_mic = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,   // Как с MAX98357A
    .dma_buf_len = 128,//64,
    .use_apll = 1,
    .fixed_mclk=-1
  };
const i2s_pin_config_t pin_config_mic = {
    .mck_io_num = -1,
  .bck_io_num = I2S_SCK,
  .ws_io_num = I2S_WS,
  .data_out_num = -1,
  .data_in_num = I2S_SD
};

//---------------------------------------------------------------------------
//--------------------------ОСН КОД-------------------------------------------------
//---------------------------------------------------------------------------

void setup() {

  Serial.begin(115200);

  i2sInit();  // Настройка микро и динамика конфигами
  xTaskCreate(task_micro, "task_micro", 1024 * 2, NULL, 1, NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}

//---------------------------------------------------------------------------
//---------------РЕАЛИЗАЦИИ-----------------------------
//---------------------------------------------------------------------------
void i2sInit(){
  // МИКРОФОН
  i2s_driver_install(I2S_PORT_MIC, &i2s_config_mic, 0, NULL);
  i2s_set_pin(I2S_PORT_MIC, &pin_config_mic);
  // ДИНАМИК
  i2s_driver_install(I2S_PORT_SPEAKER, &i2s_config_speaker, 0, NULL);        
  i2s_set_pin(I2S_PORT_SPEAKER, &pin_config_speaker);                       

}

void task_micro(void *arg)
{
  size_t bytes_read, BytesWritten;
  while(true){

    Serial.println("Microphone");
    i2s_read(I2S_PORT_MIC, (void*)i2sBuff, I2S_READ_LEN, &bytes_read, 0);

    //Serial.print("Speakers: ");Serial.print(*(TheData+(DataIdx-3)));Serial.print(*(TheData+(DataIdx-2)));Serial.print(*(TheData+(DataIdx-1)));Serial.println(*(TheData+DataIdx));
    for (;;){
      i2s_write(I2S_PORT_SPEAKER,i2sBuff+DataIdx,4,&BytesWritten,0); // Подаем по 4 семпла на MAX98357A
      DataIdx+=4;                                   // Сдвигаемся на 4 семпла вперед
      if(DataIdx>=(I2S_READ_LEN/I2S_SAMPLE_BITS)){               // Когда буфер пройден - выход из вложенного цикла
        DataIdx=0;
        break;
      }
    }

  }    
  vTaskDelete(NULL);
}


