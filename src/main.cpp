#include <Arduino.h>
#include "driver/i2s.h"
#include <string.h>
#include "nnom.h"
#include "mfcc.h"
#include "denoise_weights.h"
#include "equalizer_coeff.h" 

// ─── Аудио параметры ───
#define SAMPLE_RATE    16000
#define DMA_BUF_COUNT  4
#define DMA_BUF_LEN    64
#define GAIN           4.0f

// ─── Режим записи через Serial ───
#define SERIAL_BAUD    1000000
#define REC_DOWNSAMPLE 1                // no downsampling → full 48000 Hz
#define REC_SAMPLE_RATE (SAMPLE_RATE / REC_DOWNSAMPLE)  // 48000

static bool recording = false;

// ─── Пины ───
#define I2S_MIC_WS   GPIO_NUM_5
#define I2S_MIC_SCK  GPIO_NUM_4
#define I2S_MIC_SD   GPIO_NUM_18

#define I2S_SPK_WS   GPIO_NUM_5
#define I2S_SPK_SCK  GPIO_NUM_4
#define I2S_SPK_SD   GPIO_NUM_18

// #define I2S_SPK_WS   GPIO_NUM_21
// #define I2S_SPK_SCK  GPIO_NUM_23
// #define I2S_SPK_SD   GPIO_NUM_22

///////////////////NNNOM//////////////////////////////

#define NUM_FEATURES NUM_FILTER

#define _MAX(x, y) (((x) > (y)) ? (x) : (y))
#define _MIN(x, y) (((x) < (y)) ? (x) : (y))

#define NUM_CHANNELS 	2
#define AUDIO_FRAME_LEN 512

// audio buffer for input
float audio_buffer[AUDIO_FRAME_LEN] = {0};
int16_t audio_buffer_16bit[AUDIO_FRAME_LEN] = {0};

// buffer for output
int16_t audio_buffer_filtered[AUDIO_FRAME_LEN/2] = { 0 };


// mfcc features and their derivatives
float mfcc_feature[NUM_FEATURES] = { 0 };
float mfcc_feature_prev[NUM_FEATURES] = { 0 };
float mfcc_feature_diff[NUM_FEATURES] = { 0 };
float mfcc_feature_diff_prev[NUM_FEATURES] = { 0 };
float mfcc_feature_diff1[NUM_FEATURES] = { 0 };
// features for NN
float nn_features[64] = {0};
int8_t nn_features_q7[64] = {0};

// NN results, which is the gains for each frequency band
float band_gains[NUM_FILTER] = {0};
float band_gains_prev[NUM_FILTER] = {0};

// 0db gains coefficient
float coeff_b[NUM_FILTER][NUM_COEFF_PAIR] = FILTER_COEFF_B;
float coeff_a[NUM_FILTER][NUM_COEFF_PAIR] = FILTER_COEFF_A;
// dynamic gains coefficient
float b_[NUM_FILTER][NUM_COEFF_PAIR] = {0};

// nnom model
nnom_model_t *model;

// for microphone related data. 
volatile bool is_half_updated = false;
volatile bool is_full_updated = false;
int32_t microphone_audio_buffer[AUDIO_FRAME_LEN];


// -------------------------- I2S ---------------------

// ─── Утилиты ───
static inline int16_t clamp16(float y) {
  if (y > 32767.0f) return 32767;
  if (y < -32768.0f) return -32768;
  return (int16_t)lrintf(y);
}

static inline float normalize(int32_t value) {
  return (float)(value >> 8) / 8388608.0f;
}

// ─── I2S setup ───
void setupI2SMic() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = true,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_MIC_SCK,
    .ws_io_num = I2S_MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SD
  };
  esp_err_t err = i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  Serial.printf("[MIC] install: %s\n", esp_err_to_name(err));
  err = i2s_set_pin(I2S_NUM_0, &pins);
  Serial.printf("[MIC] set_pin: %s\n", esp_err_to_name(err));
  i2s_zero_dma_buffer(I2S_NUM_0);
}

void setupI2SSpeaker() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_SPK_SCK,
    .ws_io_num = I2S_SPK_WS,
    .data_out_num = I2S_SPK_SD,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  esp_err_t err = i2s_driver_install(I2S_NUM_1, &cfg, 0, NULL);
  Serial.printf("[SPK] install: %s\n", esp_err_to_name(err));
  err = i2s_set_pin(I2S_NUM_1, &pins);
  Serial.printf("[SPK] set_pin: %s\n", esp_err_to_name(err));
  i2s_zero_dma_buffer(I2S_NUM_1);
}


// ------------------------------ NNOM Settings ------------------------

// update the history
void y_h_update(float *y_h, uint32_t len)
{
	for (uint32_t i = len-1; i >0 ;i--)
		y_h[i] = y_h[i-1];
}

//  equalizer by multiple n order iir band pass filter. 
// y[i] = b[0] * x[i] + b[1] * x[i - 1] + b[2] * x[i - 2] - a[1] * y[i - 1] - a[2] * y[i - 2]...
void equalizer(float* x, float* y, uint32_t signal_len, float *b, float *a, uint32_t num_band, uint32_t num_order)
{
	// the y history for each band
	static float y_h[NUM_FILTER][NUM_COEFF_PAIR] = { 0 };
	static float x_h[NUM_COEFF_PAIR * 2] = { 0 };
	uint32_t num_coeff = num_order * 2 + 1;

	// i <= num_coeff (where historical x is involved in the first few points)
	// combine state and new data to get a continual x input. 
	memcpy(x_h + num_coeff, x, num_coeff * sizeof(float));
	for (uint32_t i = 0; i < num_coeff; i++)
	{
		y[i] = 0;
		for (uint32_t n = 0; n < num_band; n++)
		{
			y_h_update(y_h[n], num_coeff);
			y_h[n][0] = b[n * num_coeff] * x_h[i+ num_coeff];
			for (uint32_t c = 1; c < num_coeff; c++)
				y_h[n][0] += b[n * num_coeff + c] * x_h[num_coeff + i - c] - a[n * num_coeff + c] * y_h[n][c];
			y[i] += y_h[n][0];
		}
	}
	// store the x for the state of next round
	memcpy(x_h, &x[signal_len - num_coeff], num_coeff * sizeof(float));
	
	// i > num_coeff; the rest data not involed the x history
	for (uint32_t i = num_coeff; i < signal_len; i++)
	{
		y[i] = 0;
		for (uint32_t n = 0; n < num_band; n++)
		{
			y_h_update(y_h[n], num_coeff);
			y_h[n][0] = b[n * num_coeff] * x[i];
			for (uint32_t c = 1; c < num_coeff; c++)
				y_h[n][0] += b[n * num_coeff + c] * x[i - c] - a[n * num_coeff + c] * y_h[n][c];
			y[i] += y_h[n][0];
		}	
	}
}

// set dynamic gains. Multiple gains x b_coeff
void set_gains(float *b_in, float *b_out,  float* gains, uint32_t num_band, uint32_t num_order)
{
	uint32_t num_coeff = num_order * 2 + 1;
	for (uint32_t i = 0; i < num_band; i++)
		for (uint32_t c = 0; c < num_coeff; c++)
			b_out[num_coeff *i + c] = b_in[num_coeff * i + c] * gains[i]; 
}

void quantize_data(float*din, int8_t *dout, uint32_t size, uint32_t int_bit)
{
	float limit = (1 << int_bit); 
	for(uint32_t i=0; i<size; i++)
		dout[i] = (int8_t)(_MAX(_MIN(din[i], limit), -limit) / limit * 127);
}

void led(bool mode){
	digitalWrite(GPIO_NUM_2, (uint8_t)mode);
}


int32_t *p_new_data;
mfcc_t * mfcc;
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  setupI2SMic();
  setupI2SSpeaker();
  Serial.println("═══════════════════════════════════════");
  Serial.println("  RNN powered ESP32 hearing aid ");
  Serial.printf("  SR: %d Hz | Gain: %.1f\n", SAMPLE_RATE, GAIN);
  Serial.printf("  Serial: %d baud | Rec SR: %d Hz\n", SERIAL_BAUD, REC_SAMPLE_RATE);
  Serial.println("═══════════════════════════════════════");
  // NNOM
// NNoM model
	pinMode(GPIO_NUM_2, OUTPUT);
	model = nnom_model_create();
    // mfcc features, 0 offset, 26 bands, 512fft, 0 preempha, attached_energy_to_band0
	mfcc = mfcc_create(NUM_FEATURES, 0, NUM_FEATURES, 64*2, 0, true);

}

void loop() {
 
	size_t bytes_read;
	size_t bytes_written;
	int frames, samples;

	i2s_read(I2S_NUM_0, microphone_audio_buffer, sizeof(microphone_audio_buffer), &bytes_read, portMAX_DELAY);
	frames = bytes_read/AUDIO_FRAME_LEN;

	// move buffer (50%) overlapping, move later 50% to the first 50, then fill 
	memcpy(audio_buffer_16bit, &audio_buffer_16bit[AUDIO_FRAME_LEN/2], AUDIO_FRAME_LEN/2*sizeof(int16_t));

	// wait for new data
	while(!is_half_updated && !is_full_updated);


	if(is_half_updated){
		is_half_updated = false;
		p_new_data = microphone_audio_buffer; // i2s_reads
	}
	else
	{
		is_full_updated = false;
		p_new_data = &microphone_audio_buffer[AUDIO_FRAME_LEN/2];
	}
	// convert the file to 16bit.
	for(int i = 0; i < AUDIO_FRAME_LEN/2; i++)
		audio_buffer_16bit[AUDIO_FRAME_LEN/2+i] = SaturaLH((p_new_data[i] >> 8), -32768, 32767);

	// get mfcc
	mfcc_compute(mfcc, audio_buffer_16bit, mfcc_feature);

	// get the first and second derivative of mfcc
	for(uint32_t i=0; i< NUM_FEATURES; i++)
	{
		mfcc_feature_diff[i] = mfcc_feature[i] - mfcc_feature_prev[i];
		mfcc_feature_diff1[i] = mfcc_feature_diff[i] - mfcc_feature_diff_prev[i];
	}
	memcpy(mfcc_feature_prev, mfcc_feature, NUM_FEATURES * sizeof(float));
	memcpy(mfcc_feature_diff_prev, mfcc_feature_diff, NUM_FEATURES * sizeof(float));

	// combine MFCC with derivatives for the NN features
	memcpy(nn_features, mfcc_feature, NUM_FEATURES*sizeof(float));
	memcpy(&nn_features[NUM_FEATURES], mfcc_feature_diff, 10*sizeof(float));
	memcpy(&nn_features[NUM_FEATURES+10], mfcc_feature_diff1, 10*sizeof(float));

	// quantise them using the same scale as training data (in keras), by 2^n. 
	quantize_data(nn_features, nn_features_q7, NUM_FEATURES+20, 3);

	// run the mode with the new input
	memcpy(nnom_input_data, nn_features_q7, sizeof(nnom_input_data));
	model_run(model);

	// read the result, convert it back to float (q0.7 to float)
	for(int i=0; i< NUM_FEATURES; i++)
		band_gains[i] = (float)(nnom_output_data[i]) / 127.f;

	// one more step, limit the change of gians, to smooth the speech, per RNNoise paper
	for(int i=0; i< NUM_FEATURES; i++)
		band_gains[i] = _MAX(band_gains_prev[i]*0.8f, band_gains[i]); 
	memcpy(band_gains_prev, band_gains, NUM_FEATURES *sizeof(float));

	// update filter coefficient to applied dynamic gains to each frequency band 
	set_gains((float*)coeff_b, (float*)b_, band_gains, NUM_FILTER, NUM_ORDER);

	// convert 16bit to float for equalizer
	for (int i = 0; i < AUDIO_FRAME_LEN/2; i++)
		audio_buffer[i] = audio_buffer_16bit[i + AUDIO_FRAME_LEN / 2] / 32768.f;
			
	// finally, we apply the equalizer to this audio frame to denoise
	equalizer(audio_buffer, &audio_buffer[AUDIO_FRAME_LEN / 2], AUDIO_FRAME_LEN/2, (float*)b_,(float*)coeff_a, NUM_FILTER, NUM_ORDER);

	// convert it back to int16
	for (int i = 0; i < AUDIO_FRAME_LEN / 2; i++)
		audio_buffer_filtered[i] = audio_buffer[i + AUDIO_FRAME_LEN / 2] * 32768.f *0.6f; // 0.7 is the filter band overlapping factor

	// voice detection to show an LED PE8 -> GreenLED
	if(nnom_output_data1[0] >= 64)
		led(true);
	else
		led(false);

	// filtered frame is now in audio_buffer_filtered[], size = half_frame
	// you may implement your own method to store or playback the voice. 


	i2s_write(I2S_NUM_1, audio_buffer_filtered, frames * sizeof(int16_t), &bytes_written, portMAX_DELAY);

}