#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "time.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "esp_adc/adc_continuous.h"

// ==========================================
// CONFIGURATION
// ==========================================
const char* WIFI_SSID = "Khalid_Kashmiri";
const char* WIFI_PASS = "EloyXuxes**";
const char* TARGET_PC_IP = "192.168.1.7"; 
const int   UDP_PORT     = 4210;

// Hardware
#define MIC_PIN          8      // ADC1_CH7
#define LED_PIN          13     
#define ADC_CHANNEL      ADC_CHANNEL_7
#define ADC_UNIT         ADC_UNIT_1

// --- 40dB HARDWARE GAIN TUNING ---
#define SENSITIVITY_OFFSET 40   // TIGHTENED: Catches small voltage swings from 40dB hardware gain
#define SUSTAIN_MARGIN_DB  2.5  // DYNAMIC: Keeps recording if noise is just 2.5dB louder than quiet baseline
#define MAX_SILENCE        3    // 3 chunks (375ms) of quiet = Exit event
#define MAX_CHUNKS         80   // LIFEGUARD: Hard cap at 10 seconds (80 * 125ms)
#define SAMPLE_RATE        8000
#define CHUNK_SIZE         1000 // 125ms per chunk

// Math
const float V_REF = 3.3;
const float ADC_MAX = 4095.0;
const float MIC_SENS = 0.63; 
const float REF_DB = 94.0;

// Globals
static adc_continuous_handle_t dma_handle = nullptr;
uint8_t* dma_buf = nullptr;
uint16_t* samples = nullptr;
WiFiUDP udp;

// Variables that survive Light Sleep
RTC_DATA_ATTR int event_counter = 0; 
RTC_DATA_ATTR float baseline_p2p = 50.0f; // Dynamic moving average baseline

// ==========================================
// DMA AUDIO SAMPLING FUNCTIONS
// ==========================================
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
  dig_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2; 
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

// ==========================================
// WIFI & TIME SYNC (Boot Only)
// ==========================================
void initWiFiAndTime() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  digitalWrite(LED_PIN, HIGH); 
  
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    vTaskDelay(pdMS_TO_TICKS(500));
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    struct tm timeinfo;
    getLocalTime(&timeinfo, 10000); 
  }
  
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  digitalWrite(LED_PIN, LOW); 
}

// ==========================================
// THE FREE-RTOS SENTRY TASK
// ==========================================
void SentryTask(void *pvParameters) {
  int heartbeat_counter = 0;

  for (;;) { 
    
    // 1. VISUAL HEARTBEAT
    heartbeat_counter++;
    if (heartbeat_counter > 50) { 
      digitalWrite(LED_PIN, HIGH); 
      delayMicroseconds(1000); 
      digitalWrite(LED_PIN, LOW);
      heartbeat_counter = 0;
    }

    // 2. ULTRA-FAST SENTRY CHECK
    setCpuFrequencyMhz(80); // Bump to 80MHz to read analog pins rapidly
    int sMax = 0, sMin = 4095;
    unsigned long startT = millis();
    
    // 20ms Window: Guarantees we capture at least one full acoustic cycle down to 50Hz
    while(millis() - startT < 20) {
      int v = analogRead(MIC_PIN);
      if(v > sMax) sMax = v;
      if(v < sMin) sMin = v;
    }

    int current_p2p = sMax - sMin;

    // 3. DYNAMIC TRIGGER LOGIC
    if (current_p2p > (baseline_p2p + SENSITIVITY_OFFSET)) {
      setCpuFrequencyMhz(240); // Max power for DSP math
      digitalWrite(LED_PIN, HIGH); 
      
      // Calculate dynamic sustain threshold based on the quiet baseline
      float baseline_volts = (baseline_p2p / 2.0f) * 0.707f * (V_REF / ADC_MAX);
      float ambient_db = (baseline_volts > 0.0001f) ? (REF_DB + 20.0 * log10(baseline_volts / MIC_SENS)) : 30.0f;
      float dynamic_sustain_db = ambient_db + SUSTAIN_MARGIN_DB;

      start_dma_sampler();

      float total_db_accum = 0;
      int chunk_count = 0;
      int silence_count = 0;
      uint32_t bytes_read;
      size_t dma_buf_size = CHUNK_SIZE * sizeof(adc_digi_output_data_t);

      while (silence_count < MAX_SILENCE && chunk_count < MAX_CHUNKS) { 
        uint32_t samples_read = 0;

        while (samples_read < CHUNK_SIZE) {
          esp_err_t ret = adc_continuous_read(dma_handle, dma_buf, dma_buf_size, &bytes_read, pdMS_TO_TICKS(10));
          if (ret == ESP_OK) {
            int count = bytes_read / sizeof(adc_digi_output_data_t);
            adc_digi_output_data_t* p = (adc_digi_output_data_t*)dma_buf;
            for (int i = 0; i < count && samples_read < CHUNK_SIZE; i++) {
              samples[samples_read++] = p[i].type2.data;
            }
          }
        }
        
        long sum_raw = 0;
        for (int i = 0; i < CHUNK_SIZE; i++) { sum_raw += samples[i]; }
        
        float dc_offset = (float)sum_raw / CHUNK_SIZE;
        float sum_sq = 0;
        
        for (int i = 0; i < CHUNK_SIZE; i++) {
          float volts = ((float)samples[i] - dc_offset) * V_REF / ADC_MAX;
          sum_sq += (volts * volts);
        }
        
        float rms = sqrt(sum_sq / CHUNK_SIZE);
        float db = (rms > 0.0001) ? (REF_DB + 20.0 * log10(rms / MIC_SENS)) : 30.0;

        total_db_accum += db;
        chunk_count++;

        // Compare against the highly permissive dynamic sustain threshold
        if (db < dynamic_sustain_db) silence_count++;
        else silence_count = 0;
      }

      stop_dma_sampler(); 
      digitalWrite(LED_PIN, LOW); 

      // 4. GET TIMESTAMP & BUILD JSON
      struct tm timeinfo;
      char timestamp[30] = "Unknown Time";
      if (getLocalTime(&timeinfo, 10)) {
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &timeinfo);
      }

      // Subtract silence tail for true event duration
      float final_avg = total_db_accum / chunk_count;
      int actual_duration_ms = (chunk_count - silence_count) * 125; 
      if (actual_duration_ms < 125) actual_duration_ms = 125; 

      event_counter++;
      
      char jsonBuffer[256];
      snprintf(jsonBuffer, sizeof(jsonBuffer), 
               "{\"event\":%d, \"timestamp\":\"%s\", \"dB_SPL\":%.1f, \"dur_ms\":%d}", 
               event_counter, timestamp, final_avg, actual_duration_ms);

      // 5. TRANSMIT VIA WIFI UDP
      WiFi.mode(WIFI_STA);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      unsigned long wifiStart = millis();
      
      while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 3000) {
        vTaskDelay(pdMS_TO_TICKS(50)); 
      }

      if (WiFi.status() == WL_CONNECTED) {
        udp.beginPacket(TARGET_PC_IP, UDP_PORT);
        udp.print(jsonBuffer);
        udp.endPacket();
        
        // Success: 3 Rapid Blinks
        for(int i=0; i<3; i++) {
          digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(80));
          digitalWrite(LED_PIN, LOW);  vTaskDelay(pdMS_TO_TICKS(80));
        }
      }

      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      
      // Lifeguard Cooldown: If it maxed out at 10 seconds, sleep for 10s to prevent spam
      if (chunk_count >= MAX_CHUNKS) {
        baseline_p2p = current_p2p; 
        vTaskDelay(pdMS_TO_TICKS(10000)); 
      }

    } else {
      // DYNAMIC EMA: 95% historical, 5% new (updates slower to ignore brief ambient spikes)
      baseline_p2p = (baseline_p2p * 0.95f) + (current_p2p * 0.05f);
    }

    // 7. DEEP POWER SAVE
    setCpuFrequencyMhz(20); 
    esp_sleep_enable_timer_wakeup(80000); // Sleep for 80ms
    gpio_deep_sleep_hold_en();
    esp_light_sleep_start();
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  analogReadResolution(12);

  dma_buf = (uint8_t*)heap_caps_malloc(CHUNK_SIZE * sizeof(adc_digi_output_data_t), MALLOC_CAP_SPIRAM);
  samples = (uint16_t*)heap_caps_malloc(CHUNK_SIZE * sizeof(uint16_t), MALLOC_CAP_SPIRAM);

  if (!dma_buf || !samples) {
    while(true) { 
      digitalWrite(LED_PIN, HIGH); delay(50);
      digitalWrite(LED_PIN, LOW); delay(50);
    }
  }

  initWiFiAndTime();

  xTaskCreatePinnedToCore(SentryTask, "SentryTask", 8192, NULL, 1, NULL, 1);
  vTaskDelete(NULL); 
}

void loop() {}