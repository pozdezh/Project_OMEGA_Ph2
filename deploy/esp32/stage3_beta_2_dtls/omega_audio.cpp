#include "omega_audio.h"
#include "omega_config.h"

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <esp_task_wdt.h>

static adc_continuous_handle_t dmaHandle = nullptr;
static adc_cali_handle_t caliHandle = nullptr;
static uint8_t* dmaBuf = nullptr;
static RTC_DATA_ATTR float baselineP2pV = 0.050f;
static TaskHandle_t audioTaskHandle = nullptr;

static bool IRAM_ATTR audioConvDoneCb(adc_continuous_handle_t handle,
                                       const adc_continuous_evt_data_t* edata,
                                       void* user_data) {
  BaseType_t mustYield = pdFALSE;
  vTaskNotifyGiveFromISR(audioTaskHandle, &mustYield);
  return (mustYield == pdTRUE);
}

static void audioStartCapture() {
  audioTaskHandle = xTaskGetCurrentTaskHandle();
  adc_continuous_handle_cfg_t handle_cfg = {.max_store_buf_size = 16384, .conv_frame_size = 1024};
  adc_continuous_new_handle(&handle_cfg, &dmaHandle);
  adc_digi_pattern_config_t pattern[1] = {
      {.atten = ADC_ATTEN_DB_12, .channel = ADC_CHANNEL, .unit = ADC_UNIT, .bit_width = ADC_BITWIDTH_12}};
  adc_continuous_config_t dig_cfg = {.pattern_num = 1, .adc_pattern = pattern,
                                     .sample_freq_hz = SAMPLE_RATE,
                                     .conv_mode = ADC_CONV_SINGLE_UNIT_1,
                                     .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2};
  adc_continuous_config(dmaHandle, &dig_cfg);
  adc_continuous_evt_cbs_t cbs = {.on_conv_done = audioConvDoneCb};
  adc_continuous_register_event_callbacks(dmaHandle, &cbs, nullptr);
  adc_continuous_start(dmaHandle);
}

static void audioStopCapture() {
  if (dmaHandle) {
    adc_continuous_stop(dmaHandle);
    adc_continuous_deinit(dmaHandle);
    dmaHandle = nullptr;
  }
}

static float audioCaptureChunkDb() {
  uint32_t samples_read = 0;
  uint32_t bytes_read;
  size_t dma_buf_size = CHUNK_SIZE * sizeof(adc_digi_output_data_t);
  int timeout_fails = 0;
  float mean = 0.0f;
  float M2 = 0.0f;

  while (samples_read < CHUNK_SIZE) {
    esp_task_wdt_reset();
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) == 0) {
      timeout_fails++;
      if (timeout_fails > 3) break;
      continue;
    }
    timeout_fails = 0;
    while (samples_read < CHUNK_SIZE &&
           adc_continuous_read(dmaHandle, dmaBuf, dma_buf_size, &bytes_read, 0) == ESP_OK) {
      int count = bytes_read / sizeof(adc_digi_output_data_t);
      adc_digi_output_data_t* p = (adc_digi_output_data_t*)dmaBuf;
      for (int i = 0; i < count && samples_read < CHUNK_SIZE; i++) {
        int mv = 0;
        adc_cali_raw_to_voltage(caliHandle, p[i].type2.data, &mv);
        float v = mv / 1000.0f;
        samples_read++;
        float delta = v - mean;
        mean += delta / samples_read;
        float delta2 = v - mean;
        M2 += delta * delta2;
      }
    }
  }
  if (samples_read < 2) return 30.0f;
  float variance = M2 / samples_read;
  float rms = sqrt(variance);
  return (rms > 0.0001f) ? (REF_DB + 20.0f * log10(rms / MIC_SENS)) : 30.0f;
}

bool audioInit() {
  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT, .chan = ADC_CHANNEL,
      .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12,
  };
  if (adc_cali_create_scheme_curve_fitting(&cali_config, &caliHandle) != ESP_OK) {
    return false;
  }
  dmaBuf = (uint8_t*)heap_caps_malloc(CHUNK_SIZE * sizeof(adc_digi_output_data_t),
                                      MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  return dmaBuf != nullptr;
}

float audioAmbientPoll() {
  int sMax = 0, sMin = 4095;
  unsigned long startT = millis();
  while (millis() - startT < AMBIENT_POLL_MS) {
    int v = analogRead(MIC_PIN);
    if (v > sMax) sMax = v;
    if (v < sMin) sMin = v;
    taskYIELD();
  }
  int max_mv = 0, min_mv = 0;
  adc_cali_raw_to_voltage(caliHandle, sMax, &max_mv);
  adc_cali_raw_to_voltage(caliHandle, sMin, &min_mv);
  return (max_mv - min_mv) / 1000.0f;
}

float audioAmbientDb() {
  float baseline_rms = (baselineP2pV / 2.0f) * 0.707f;
  return (baseline_rms > 0.0001f) ? (REF_DB + 20.0f * log10(baseline_rms / MIC_SENS)) : 30.0f;
}

float audioTriggerThresholdV(float ambient_db) {
  float target_wake_rms = MIC_SENS * pow(10.0f, ((ambient_db + WAKEUP_MARGIN_DB) - REF_DB) / 20.0f);
  return (target_wake_rms * 2.0f) / 0.707f;
}

void audioUpdateBaseline(float current_p2p_v) {
  baselineP2pV = (baselineP2pV * 0.95f) + (current_p2p_v * 0.05f);
}

void audioResetBaseline(float current_p2p_v) {
  baselineP2pV = current_p2p_v;
}

AudioCaptureResult audioCaptureSyncMeasurement() {
  audioStartCapture();
  float total = 0.0f;
  for (int c = 0; c < SYNC_MEASUREMENT_CHUNKS; c++) {
    total += audioCaptureChunkDb();
    vTaskDelay(1);
  }
  audioStopCapture();

  AudioCaptureResult result;
  result.db = total / (float)SYNC_MEASUREMENT_CHUNKS;
  result.duration_s = 1.0f;
  result.hit_max_chunks = false;
  result.silence_tail_s = 0.0f;
  return result;
}

AudioCaptureResult audioCaptureTriggeredEvent(float ambient_db) {
  float dynamicSustainDb = ambient_db + SUSTAIN_MARGIN_DB;

  audioStartCapture();
  float total_db_accum = 0.0f;
  float silence_db_accum = 0.0f;
  int chunk_count = 0;
  int silence_count = 0;

  while (silence_count < MAX_SILENCE && chunk_count < MAX_CHUNKS) {
    float chunk_db = audioCaptureChunkDb();
    total_db_accum += chunk_db;
    chunk_count++;
    if (chunk_db < dynamicSustainDb) {
      silence_count++;
      silence_db_accum += chunk_db;
    } else {
      silence_count = 0;
      silence_db_accum = 0.0f;
    }
    vTaskDelay(1);
  }
  audioStopCapture();

  int active = chunk_count - silence_count;
  float final_avg = (active > 0) ? ((total_db_accum - silence_db_accum) / active)
                                 : (total_db_accum / chunk_count);
  float actual_duration_s = ((active > 0) ? (active * 125) : (chunk_count * 125)) / 1000.0f;
  // Only meaningful when the loop stopped BECAUSE it confirmed silence
  // (active > 0, silence_count == MAX_SILENCE). If it stopped by hitting
  // MAX_CHUNKS instead, the sound may still have been ongoing - there is no
  // confirmed quiet tail to subtract, so the loop-end time IS the best
  // available estimate of when the sound stopped.
  float silence_tail_s = (active > 0) ? (silence_count * 125) / 1000.0f : 0.0f;

  AudioCaptureResult result;
  result.db = final_avg;
  result.duration_s = actual_duration_s;
  result.hit_max_chunks = (chunk_count >= MAX_CHUNKS);
  result.silence_tail_s = silence_tail_s;
  return result;
}
