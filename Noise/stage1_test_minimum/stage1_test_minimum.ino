#include <Arduino.h>
#include "esp_sleep.h"
#include "esp_adc/adc_continuous.h"
#include <WiFi.h>

// --- HARDWARE WIRING ---
#define MIC_PIN          8      // MAX9814 OUT (ADC1_CH7)
#define LED_PIN          13     // Visual Feedback LED
#define ADC_CHANNEL      ADC_CHANNEL_7
#define ADC_UNIT         ADC_UNIT_1

// --- TUNING CONSTANTS ---
#define WAKE_THRESHOLD   400    // Raw ADC delta to trigger DMA
#define SUSTAIN_DB       58.0   // Drop below this = stop recording
#define MAX_SILENCE      3      // 3 chunks (375ms) of quiet = Exit
#define SAMPLE_RATE      8000
#define CHUNK_SIZE       1000   // 125ms per chunk

// --- MATH CONSTANTS ---
const float V_REF = 3.3;
const float ADC_MAX = 4095.0;
const float SENSITIVITY = 0.63; 
const float REF_DB = 94.0;

// Variables that survive Light Sleep
RTC_DATA_ATTR int heartbeat_counter = 0;
RTC_DATA_ATTR int event_counter = 0;

static adc_continuous_handle_t dma_handle = nullptr;

// --- PSRAM AUDIO BUFFERS ---
uint8_t* dma_buf = nullptr;
uint16_t* samples = nullptr;

// ==============================================================================
// ESP32-S3 CONTINUOUS ADC SETUP
// ==============================================================================
void start_dma_sampler() {
  adc_continuous_handle_cfg_t handle_cfg = {
    .max_store_buf_size = 4096,
    .conv_frame_size = 1024
  };
  adc_continuous_new_handle(&handle_cfg, &dma_handle);

  adc_digi_pattern_config_t pattern[1];
  pattern[0].atten     = ADC_ATTEN_DB_12; 
  pattern[0].channel   = ADC_CHANNEL;
  pattern[0].unit      = ADC_UNIT;
  pattern[0].bit_width = ADC_BITWIDTH_12;

  adc_continuous_config_t dig_cfg = {};
  dig_cfg.sample_freq_hz = SAMPLE_RATE;
  dig_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  dig_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2; // Type 2 is mandatory for S3
  dig_cfg.adc_pattern = pattern;
  dig_cfg.pattern_num = 1;

  adc_continuous_config(dma_handle, &dig_cfg);
  adc_continuous_start(dma_handle);
}

void stop_dma_sampler() {
  if (dma_handle) {
    adc_continuous_stop(dma_handle);
    adc_continuous_deinit(dma_handle);
    dma_handle = nullptr;
  }
}

// ==============================================================================
// MAIN SETUP
// ==============================================================================
void setup() {
  // Disable WiFi to prevent baseline power draw
  WiFi.mode(WIFI_OFF); 
  
  pinMode(LED_PIN, OUTPUT);
  analogReadResolution(12);

  // --- ALLOCATE HEAVY BUFFERS TO PSRAM ---
  size_t dma_buf_size = CHUNK_SIZE * sizeof(adc_digi_output_data_t);
  size_t samples_size = CHUNK_SIZE * sizeof(uint16_t);
  
  dma_buf = (uint8_t*)heap_caps_malloc(dma_buf_size, MALLOC_CAP_SPIRAM);
  samples = (uint16_t*)heap_caps_malloc(samples_size, MALLOC_CAP_SPIRAM);

  // Fatal Error Check: If PSRAM fails, strobe the LED forever
  if (!dma_buf || !samples) {
    while(true) {
      digitalWrite(LED_PIN, HIGH); delay(50);
      digitalWrite(LED_PIN, LOW); delay(50);
    }
  }

  // Boot Confirmation: Long solid light means PSRAM is locked and loaded
  digitalWrite(LED_PIN, HIGH); 
  delay(1000); 
  digitalWrite(LED_PIN, LOW);
}

// ==============================================================================
// MAIN LOOP
// ==============================================================================
void loop() {
  // --- 1. VISUAL HEARTBEAT ---
  heartbeat_counter++;
  if (heartbeat_counter > 40) { 
    // 1 millisecond flash every ~2 seconds
    digitalWrite(LED_PIN, HIGH); 
    delayMicroseconds(1000); 
    digitalWrite(LED_PIN, LOW);
    heartbeat_counter = 0;
  }

  // --- 2. FAST SENTRY CHECK (80MHz) ---
  setCpuFrequencyMhz(80); 
  
  int sMax = 0, sMin = 4095;
  unsigned long startT = millis();
  
  // 20ms analogRead burst
  while(millis() - startT < 20) {
    int v = analogRead(MIC_PIN);
    if(v > sMax) sMax = v;
    if(v < sMin) sMin = v;
  }

  // --- 3. TRIGGER DETECTED ---
  if ((sMax - sMin) > WAKE_THRESHOLD) {
    setCpuFrequencyMhz(240); // Max power for accurate DSP math
    digitalWrite(LED_PIN, HIGH); // Solid light: Actively sampling
    
    start_dma_sampler();

    float total_db_accum = 0;
    int chunk_count = 0;
    int silence_count = 0;
    uint32_t bytes_read;
    size_t dma_buf_size = CHUNK_SIZE * sizeof(adc_digi_output_data_t);

    // DMA Sampling Loop
    while (silence_count < MAX_SILENCE && chunk_count < 80) { // Max 10s
      
      uint32_t samples_read = 0;

      // Drain DMA buffer until we have 1 chunk (125ms)
      while (samples_read < CHUNK_SIZE) {
        esp_err_t ret = adc_continuous_read(dma_handle, dma_buf, dma_buf_size, &bytes_read, pdMS_TO_TICKS(10));
        if (ret == ESP_OK) {
          int count = bytes_read / sizeof(adc_digi_output_data_t);
          adc_digi_output_data_t* p = (adc_digi_output_data_t*)dma_buf;
          for (int i = 0; i < count && samples_read < CHUNK_SIZE; i++) {
             // Type 2 data struct for S3
            samples[samples_read++] = p[i].type2.data;
          }
        }
      }
      
      long sum_raw = 0;
      for (int i = 0; i < CHUNK_SIZE; i++) {
        sum_raw += samples[i];
      }
      
      float dc_offset = (float)sum_raw / CHUNK_SIZE;
      float sum_sq = 0;
      
      for (int i = 0; i < CHUNK_SIZE; i++) {
        float volts = ((float)samples[i] - dc_offset) * V_REF / ADC_MAX;
        sum_sq += (volts * volts);
      }
      
      float rms = sqrt(sum_sq / CHUNK_SIZE);
      float db = (rms > 0.0001) ? (REF_DB + 20.0 * log10(rms / SENSITIVITY)) : 30.0;

      total_db_accum += db;
      chunk_count++;

      if (db < SUSTAIN_DB) silence_count++;
      else silence_count = 0;
    }

    // Instantly kill DMA hardware to stop power drain
    stop_dma_sampler(); 

    // --- 4. CONSTRUCT JSON ---
    float final_avg = total_db_accum / chunk_count;
    int duration = chunk_count * 125; // 125ms per chunk
    event_counter++;
    
    // Construct JSON entirely in memory
    char jsonBuffer[128];
    snprintf(jsonBuffer, sizeof(jsonBuffer), 
             "{\"event\":%d, \"dBA\":%.1f, \"dur_ms\":%d}", 
             event_counter, final_avg, duration);

    // --- 5. VISUAL SUCCESS CONFIRMATION ---
    digitalWrite(LED_PIN, LOW);  // Turn off solid light
    delay(200);                  // Short dark pause

    // 3 distinct blinks to confirm JSON is built!
    for(int i=0; i<3; i++) {
      digitalWrite(LED_PIN, HIGH); delay(100);
      digitalWrite(LED_PIN, LOW);  delay(100);
    }
  }

  // --- 6. RETURN TO MAX ENERGY SAVE ---
  setCpuFrequencyMhz(20); // Drop clock down before sleep
  esp_sleep_enable_timer_wakeup(50000); // Sleep for 50ms
  esp_light_sleep_start();
}