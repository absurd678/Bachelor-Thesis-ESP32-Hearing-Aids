#pragma once

#include <algorithm>
#include <cstdio>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "audio_config.h"
#include "audio_tools.h"
#include "norm_stats.h"
#include "rnn_bands_model.h"
#include "tensorflow/lite/core/c/common.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

#ifndef MODEL_DATA
#define MODEL_DATA _content_drive_MyDrive_Colab_RNN_export_rnn_bands_int8_micro_tflite
#endif

static constexpr int kSeqLen = 8;
static constexpr int kHopSize = 256;
static constexpr int kBands = 8;
static constexpr int kFeatures = 18;
static constexpr int kMaxRealtimeOutput = AUDIO_BUF_LEN + kHopSize;

static constexpr int kBandBins[kBands + 1] = {
    2, 3, 6, 11, 17, 26, 36, 53, 85
};

static float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

static void die(const char* message) {
  printf("%s\n", message);
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void ensure_tflite_ok(TfLiteStatus status, const char* message) {
  if (status != kTfLiteOk) {
    die(message);
  }
}

static uint8_t* alloc_internal_ram_aligned(size_t size, size_t alignment) {
  const size_t padded_size = size + alignment - 1;
  void* raw = heap_caps_malloc(padded_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!raw) {
    printf("Internal RAM alloc failed: %zu bytes\n", padded_size);
    die("Internal RAM alloc failed");
  }

  const uintptr_t raw_addr = reinterpret_cast<uintptr_t>(raw);
  const uintptr_t aligned_addr = (raw_addr + alignment - 1) & ~(uintptr_t)(alignment - 1);
  uint8_t* aligned = reinterpret_cast<uint8_t*>(aligned_addr);
  memset(aligned, 0, size);
  return aligned;
}

static int8_t quantize_int8(float x, const TfLiteQuantizationParams& q) {
  const int32_t v = (int32_t)lrintf(x / q.scale + q.zero_point);
  if (v < -128) return -128;
  if (v > 127) return 127;
  return (int8_t)v;
}

static float dequantize_int8(int8_t x, const TfLiteQuantizationParams& q) {
  return ((float)x - (float)q.zero_point) * q.scale;
}

static TfLiteQuantizationParams read_tensor_quant_params(
    const tflite::Tensor* tensor,
    const char* name
) {
  if (!tensor || tensor->type() != tflite::TensorType_INT8) {
    // Serial.printf("%s tensor in flatbuffer is not INT8\n", name);
    die("Invalid tensor type");
  }

  const tflite::QuantizationParameters* quant = tensor->quantization();
  if (!quant || !quant->scale() || quant->scale()->size() == 0 ||
      !quant->zero_point() || quant->zero_point()->size() == 0) {
    printf("%s tensor has no quantization parameters\n", name);
    die("Missing quantization parameters");
  }

  TfLiteQuantizationParams params;
  params.scale = quant->scale()->Get(0);
  params.zero_point = (int32_t)quant->zero_point()->Get(0);
  return params;
}

static size_t tensor_element_count(const TfLiteTensor* tensor) {
  return tensor ? tensor->bytes : 0;
}

static void compute_bands(const ComplexSample* spec, float bands[kBands]) {
  for (int b = 0; b < kBands; ++b) {
    const int k0 = kBandBins[b];
    int k1 = kBandBins[b + 1];
    if (k1 <= k0) {
      k1 = k0 + 1;
    }

    float sum = 0.0f;
    int count = 0;
    for (int k = k0; k < k1; ++k) {
      const float power = spec[k].real * spec[k].real + spec[k].imag * spec[k].imag;
      sum += power;
      ++count;
    }
    bands[b] = sum / (float)count;
  }
}

static void compute_features(
    const float bands[kBands],
    bool* has_prev_log_bands,
    float prev_log_bands[kBands],
    float features[kFeatures]
) {
  float log_e[kBands];
  float positive_flux = 0.0f;

  for (int b = 0; b < kBands; ++b) {
    log_e[b] = log10f(bands[b] + 1e-8f);
    features[b] = log_e[b];

    float delta = 0.0f;
    if (*has_prev_log_bands) {
      delta = log_e[b] - prev_log_bands[b];
      if (delta > 0.0f) {
        positive_flux += delta;
      }
    }
    features[kBands + b] = delta;
  }

  float log_sum = 0.0f;
  float arith_sum = 0.0f;
  for (int b = 0; b < kBands; ++b) {
    log_sum += logf(bands[b] + 1e-8f);
    arith_sum += bands[b];
  }

  const float geo_mean = expf(log_sum / (float)kBands);
  const float arith_mean = arith_sum / (float)kBands;
  features[16] = geo_mean / (arith_mean + 1e-8f);
  features[17] = positive_flux;

  for (int i = 0; i < kFeatures; ++i) {
    features[i] = (features[i] - kNormMean[i]) / kNormStd[i];
  }

  memcpy(prev_log_bands, log_e, sizeof(log_e));
  *has_prev_log_bands = true;
}

static void update_history(float history[kSeqLen][kFeatures], const float current[kFeatures]) {
    for (int t = kSeqLen - 1; t > 0; --t) {
        for (int f = 0; f < kFeatures; ++f) {
            history[t][f] = history[t - 1][f];
        }
    }

    for (int f = 0; f < kFeatures; ++f) {
        history[0][f] = current[f];
    }
}

static int bin_to_band(int bin_index) {
  for (int b = 0; b < kBands; ++b) {
    if (bin_index >= kBandBins[b] && bin_index < kBandBins[b + 1]) {
      return b;
    }
  }
  return -1;
}

static void apply_gains(ComplexSample* spec, const float gains[kBands]) {
  for (int k = 0; k <= FFT_SIZE / 2; ++k) {
    const int band = bin_to_band(k);
    const float gain = band >= 0 ? gains[band] : 0.2f;

    spec[k].real *= gain;
    spec[k].imag *= gain;

    if (k > 0 && k < FFT_SIZE / 2) {
      const int mirror = FFT_SIZE - k;
      spec[mirror].real = spec[k].real;
      spec[mirror].imag = -spec[k].imag;
    }
  }
}

class RnnBandModel {
public:
  RnnBandModel()
      : model_(nullptr),
        interpreter_(nullptr),
        input_(nullptr),
        output_(nullptr),
        input_count_(0),
        output_count_(0),
        input_data_(nullptr),
        output_data_(nullptr),
        tensor_arena_(nullptr) {
  }

  void init() {
    if (!tensor_arena_) {
      tensor_arena_ = alloc_internal_ram_aligned(kTensorArenaSize, 16);
    } else {
      memset(tensor_arena_, 0, kTensorArenaSize);
    }

    model_ = tflite::GetModel(MODEL_DATA);
    if (!model_) {
      die("GetModel failed");
    }
    if (model_->version() != TFLITE_SCHEMA_VERSION) {
      printf("Unsupported model schema version: %lu, expected: %d\n",
             model_->version(), TFLITE_SCHEMA_VERSION);
      die("Unsupported model schema version");
    }

    register_ops();

    interpreter_ = new (interpreter_storage_) tflite::MicroInterpreter(
        model_,
        resolver_,
        tensor_arena_,
        kTensorArenaSize
    );

    ensure_tflite_ok(interpreter_->AllocateTensors(), "AllocateTensors failed");

    input_ = interpreter_->input(0);
    output_ = interpreter_->output(0);
    if (!input_ || !output_) {
      die("Cannot get model input/output tensors");
    }
    // if (input_->type != kTfLiteInt8 || output_->type != kTfLiteInt8) {
    //   Serial.println("Bad tensor types:");
    //   Serial.printf("input tensor addr=%p type=%d bytes=%zu data=%p scale=%g zp=%d\n",
    //                 input_,
    //                 input_->type,
    //                 input_->bytes,
    //                 input_->data.int8,
    //                 input_->params.scale,
    //                 input_->params.zero_point);

    //   Serial.printf("output tensor addr=%p type=%d bytes=%zu data=%p scale=%g zp=%d\n",
    //                 output_,
    //                 output_->type,
    //                 output_->bytes,
    //                 output_->data.int8,
    //                 output_->params.scale,
    //                 output_->params.zero_point);

    //   die("Runtime tensor type check failed");
    // }

    const tflite::SubGraph* subgraph = model_->subgraphs()->Get(0);
    const tflite::Tensor* input_tensor =
        subgraph->tensors()->Get(subgraph->inputs()->Get(0));
    const tflite::Tensor* output_tensor =
        subgraph->tensors()->Get(subgraph->outputs()->Get(0));

    input_quant_ = read_tensor_quant_params(input_tensor, "Input");
    output_quant_ = read_tensor_quant_params(output_tensor, "Output");
    input_count_ = tensor_element_count(input_);
    output_count_ = tensor_element_count(output_);
    input_data_ = input_->data.int8;
    output_data_ = output_->data.int8;

    if (!input_data_ || !output_data_ || input_count_ == 0 || output_count_ == 0) {
      die("Invalid model input/output tensor data");
    }

    printf("RNN initialized\n");
    printf("Arena size: %zu bytes\n", (size_t)kTensorArenaSize);
    printf("Arena addr: %p\n", tensor_arena_);
    printf("Arena used: %zu bytes\n", interpreter_->arena_used_bytes());
  }

  void run(const float history[kSeqLen][kFeatures], float gains[kBands]) {
    int8_t* input_data = (int8_t*)input_data_;
    size_t idx = 0;

    for (int t = 0; t < kSeqLen; ++t) {
      for (int f = 0; f < kFeatures; ++f) {
        if (idx < input_count_) {
          input_data[idx++] = quantize_int8(history[t][f], input_quant_);
        }
      }
    }
    while (idx < input_count_) {
      input_data[idx++] = quantize_int8(0.0f, input_quant_);
    }
    // ------------ Debug output -------------
    printf("input type=%d bytes=%zu ptr=%p\n",
           input_->type, input_->bytes, input_->data.int8);

    printf("output type=%d bytes=%zu ptr=%p\n",
           output_->type, output_->bytes, output_->data.int8);

    printf("input scale=%g zp=%ld\n",
           input_->params.scale, input_->params.zero_point);

    printf("output scale=%g zp=%ld\n",
           output_->params.scale, output_->params.zero_point);

    printf("arena used=%zu / %d\n",
           interpreter_->arena_used_bytes(), kTensorArenaSize);

    printf("stack watermark=%u\n",
           uxTaskGetStackHighWaterMark(NULL));

    // if (input_->type != kTfLiteInt8) {
    //   die("Runtime input tensor is not int8");
    // }

    // if (output_->type != kTfLiteInt8) {
    //   die("Runtime output tensor is not int8");
    // }

    if (input_count_ != (size_t)(kSeqLen * kFeatures)) {
      printf("Bad input size: expected %d, got %zu\n",
             kSeqLen * kFeatures,
             input_count_);
      die("Bad input tensor size");
    }

    if (output_count_ != (size_t)kBands) {
      printf("Bad output size: expected %d, got %zu\n",
             kBands,
             output_count_);
      die("Bad output tensor size");
    }

    if (input_quant_.scale == 0.0f || output_quant_.scale == 0.0f) {
      die("Invalid int8 quantization scale");
    }

// -------------- Invoke ------------------
    ensure_tflite_ok(interpreter_->Invoke(), "Invoke failed");

    const int8_t* output_data = (const int8_t*)output_data_;
    for (int b = 0; b < kBands; ++b) {
      const size_t src = output_count_ > 0 ? std::min((size_t)b, output_count_ - 1) : 0;
      gains[b] = clamp01(dequantize_int8(output_data[src], output_quant_));
    }
  }

private:
  void register_ops() {
    ensure_tflite_ok(resolver_.AddShape(), "AddShape failed");
    ensure_tflite_ok(resolver_.AddReshape(), "AddReshape failed");
    ensure_tflite_ok(resolver_.AddStridedSlice(), "AddStridedSlice failed");
    ensure_tflite_ok(resolver_.AddTranspose(), "AddTranspose failed");
    ensure_tflite_ok(resolver_.AddUnpack(), "AddUnpack failed");
    ensure_tflite_ok(resolver_.AddPack(), "AddPack failed");
    ensure_tflite_ok(resolver_.AddFill(), "AddFill failed");
    ensure_tflite_ok(resolver_.AddFullyConnected(), "AddFullyConnected failed");
    ensure_tflite_ok(resolver_.AddSplit(), "AddSplit failed");
    ensure_tflite_ok(resolver_.AddConcatenation(), "AddConcatenation failed");
    ensure_tflite_ok(resolver_.AddAdd(), "AddAdd failed");
    ensure_tflite_ok(resolver_.AddLogistic(), "AddLogistic failed");
    ensure_tflite_ok(resolver_.AddMul(), "AddMul failed");
    ensure_tflite_ok(resolver_.AddSub(), "AddSub failed");
    ensure_tflite_ok(resolver_.AddTanh(), "AddTanh failed");
  }

  const tflite::Model* model_;
  tflite::MicroMutableOpResolver<15> resolver_;
  tflite::MicroInterpreter* interpreter_;
  TfLiteTensor* input_;
  TfLiteTensor* output_;
  TfLiteQuantizationParams input_quant_;
  TfLiteQuantizationParams output_quant_;
  size_t input_count_;
  size_t output_count_;
  void* input_data_;
  void* output_data_;
  uint8_t* tensor_arena_;
  alignas(tflite::MicroInterpreter) uint8_t interpreter_storage_[sizeof(tflite::MicroInterpreter)];
};

class RealtimeRnnFilter {
public:
  RealtimeRnnFilter()
      : input_size_(0),
        has_prev_log_bands_(false),
        frame_count_(0) {
  }

  void init() {
    make_hann(window_);
    memset(input_, 0, sizeof(input_));
    memset(ola_, 0, sizeof(ola_));
    memset(ola_norm_, 0, sizeof(ola_norm_));
    memset(history_, 0, sizeof(history_));
    memset(prev_log_bands_, 0, sizeof(prev_log_bands_));
    memset(spec, 0, sizeof(spec));
    memset(bands, 0, sizeof(bands));
    memset(features, 0, sizeof(features));
    memset(gains, 0, sizeof(gains));
    input_size_ = 0;
    has_prev_log_bands_ = false;
    frame_count_ = 0;
    model_.init();
  }

  size_t process(const float* samples, size_t count, int16_t* output, size_t output_capacity) {
    append_input(samples, count);

    size_t produced = 0;
    while (input_size_ >= FFT_SIZE && produced + kHopSize <= output_capacity) {
      process_frame();
      emit_hop(&output[produced]);
      produced += kHopSize;
      consume_hop();
    }

    return produced;
  }

  size_t frame_count() const {
    return frame_count_;
  }

private:
  static constexpr size_t kInputCapacity = FFT_SIZE + AUDIO_BUF_LEN + kHopSize;
  static constexpr size_t kOlaCapacity = FFT_SIZE;

  void append_input(const float* samples, size_t count) {
    if (count > kInputCapacity - input_size_) {
      count = kInputCapacity - input_size_;
    }
    memcpy(&input_[input_size_], samples, count * sizeof(float));
    input_size_ += count;
  }

  void process_frame() {
    

    for (int i = 0; i < FFT_SIZE; ++i) {
      spec[i].real = input_[i] * window_[i];
      spec[i].imag = 0.0f;
    }

    fft(spec, FFT_SIZE, false);

    
    compute_bands(spec, bands);

    
    compute_features(bands, &has_prev_log_bands_, prev_log_bands_, features); //prev_log_bands_? 
    update_history(history_, features);

    printf("loop stack watermark before RNN: %u\n",
           uxTaskGetStackHighWaterMark(NULL));
    model_.run(history_, gains);
    

    if (frame_count_ % 1000 == 0) {
      printf("frame %zu gains:", frame_count_);
      for (int b = 0; b < kBands; ++b) {
        printf(" %.3f", gains[b]);
      }
      printf("\n");
    }

    apply_gains(spec, gains);
    fft(spec, FFT_SIZE, true);

    for (int i = 0; i < FFT_SIZE; ++i) {
      ola_[i] += spec[i].real * window_[i];
      ola_norm_[i] += window_[i] * window_[i];
    }

    ++frame_count_;
  }

  void emit_hop(int16_t* output) {
    for (int i = 0; i < kHopSize; ++i) {
      float y = ola_[i];
      if (ola_norm_[i] > 1e-8f) {
        y /= ola_norm_[i];
      }
      output[i] = clamp16(y * 32767.0f);
    }
  }

  void consume_hop() {
    memmove(input_, &input_[kHopSize], (input_size_ - kHopSize) * sizeof(float));
    input_size_ -= kHopSize;

    memmove(ola_, &ola_[kHopSize], (kOlaCapacity - kHopSize) * sizeof(float));
    memmove(ola_norm_, &ola_norm_[kHopSize], (kOlaCapacity - kHopSize) * sizeof(float));
    memset(&ola_[kOlaCapacity - kHopSize], 0, kHopSize * sizeof(float));
    memset(&ola_norm_[kOlaCapacity - kHopSize], 0, kHopSize * sizeof(float));
  }

  RnnBandModel model_;
  float window_[FFT_SIZE];
  ComplexSample spec[FFT_SIZE];
  float bands[kBands];
  float features[kFeatures];
  float gains[kBands];
  float input_[kInputCapacity];
  float ola_[kOlaCapacity];
  float ola_norm_[kOlaCapacity];
  float history_[kSeqLen][kFeatures];
  float prev_log_bands_[kBands];
  size_t input_size_;
  bool has_prev_log_bands_;
  size_t frame_count_;
};
