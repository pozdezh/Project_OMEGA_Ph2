#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "time.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// ==========================================
// CONFIGURATION
// ==========================================
const char* WIFI_SSID = "Khalid_Kashmiri";
const char* WIFI_PASS = "EloyXuxes**";
const char* TARGET_PC_IP = "192.168.1.7"; 
const int   UDP_PORT     = 4210;

// --- STATIC IP TO BYPASS DHCP DELAY ---
IPAddress local_IP(192, 168, 1, 150); // The fixed IP for this ESP32
IPAddress gateway(192, 168, 1, 1);    // Your router's IP
IPAddress subnet(255, 255, 255, 0);

// Hardware
#define MIC_PIN          8      // ADC1_CH7
#define LED_PIN          13     
#define ADC_CHANNEL      ADC_CHANNEL_7
#define ADC_UNIT         ADC_UNIT_1

// --- DYNAMIC ACOUSTIC TUNING (Pure dB Margins) ---
#define WAKEUP_MARGIN_DB     6.0f    // WAKE-UP: Trigger if peak sound is 6.0 dB louder than ambient
#define SUSTAIN_MARGIN_DB    1.5f    // SUSTAIN: 1.5 dB. Forgiving for speech gaps.
#define MAX_SILENCE          4       // HYSTERESIS: Reduced to 4 chunks (500ms) for faster exits
#define MAX_CHUNKS           80      // Hard cap at 10 seconds
#define SAMPLE_RATE          8000
#define CHUNK_SIZE           1000    // 125ms per chunk

// Acoustic Math
const float MIC_SENS = 0.63f; 
const float REF_DB = 94.0f;

// Globals
static adc_continuous_handle_t dma_handle = nullptr;
static adc_cali_handle_t cali_handle = nullptr;

uint8_t* dma_buf = nullptr;
WiFiUDP udp;

// RTC Variables (Survive Light Sleep)
RTC_DATA_ATTR int event_counter = 0; 
RTC_DATA_ATTR float baseline_p2p_v = 0.050f; 

// ==========================================
// FACTORY EFUSE CALIBRATION
// ==========================================
bool init_adc_calibration() {
  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT,
      .chan = ADC_CHANNEL,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_12,
  };
  esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);
  return (ret == ESP_OK);
}

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
  WiFi.config(local_IP, gateway, subnet); // Fast boot
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
    // 1. VISUAL HEARTBEAT (Adjusted to tick roughly every 4 seconds)
    heartbeat_counter++;
    if (heartbeat_counter > 26) { 
      digitalWrite(LED_PIN, HIGH); 
      delayMicroseconds(1000); 
      digitalWrite(LED_PIN, LOW);
      heartbeat_counter = 0;
    }

    // 2. PASSIVE STATE: ULTRA-FAST SENTRY CHECK
    setCpuFrequencyMhz(80); 
    int sMax = 0, sMin = 4095;
    unsigned long startT = millis();
    
    while(millis() - startT < 20) {
      int v = analogRead(MIC_PIN);
      if(v > sMax) sMax = v;
      if(v < sMin) sMin = v;
    }

    int max_mv = 0, min_mv = 0;
    adc_cali_raw_to_voltage(cali_handle, sMax, &max_mv);
    adc_cali_raw_to_voltage(cali_handle, sMin, &min_mv);
    float current_p2p_v = (max_mv - min_mv) / 1000.0f; 

    // --- DYNAMIC BASELINE MATH ---
    float baseline_rms = (baseline_p2p_v / 2.0f) * 0.707f; 
    float ambient_db = (baseline_rms > 0.0001f) ? (REF_DB + 20.0f * log10(baseline_rms / MIC_SENS)) : 30.0f;
    
    float target_wake_db = ambient_db + WAKEUP_MARGIN_DB;
    float dynamic_sustain_db = ambient_db + SUSTAIN_MARGIN_DB;

    float target_wake_rms = MIC_SENS * pow(10.0f, (target_wake_db - REF_DB) / 20.0f);
    float trigger_threshold_v = (target_wake_rms * 2.0f) / 0.707f;

    // 3. WAKE-UP LOGIC
    if (current_p2p_v > trigger_threshold_v) {
      
      setCpuFrequencyMhz(160); // Overclock for DSP
      digitalWrite(LED_PIN, HIGH); 

      start_dma_sampler();

      float total_db_accum = 0;
      float silence_db_accum = 0; 
      int chunk_count = 0;
      int silence_count = 0;
      uint32_t bytes_read;
      size_t dma_buf_size = CHUNK_SIZE * sizeof(adc_digi_output_data_t);

      while (silence_count < MAX_SILENCE && chunk_count < MAX_CHUNKS) { 
        uint32_t samples_read = 0;
        float sum_volts = 0;
        float volts_array[CHUNK_SIZE];

        while (samples_read < CHUNK_SIZE) {
          esp_err_t ret = adc_continuous_read(dma_handle, dma_buf, dma_buf_size, &bytes_read, pdMS_TO_TICKS(10));
          if (ret == ESP_OK) {
            int count = bytes_read / sizeof(adc_digi_output_data_t);
            adc_digi_output_data_t* p = (adc_digi_output_data_t*)dma_buf;
            
            for (int i = 0; i < count && samples_read < CHUNK_SIZE; i++) {
              int raw_val = p[i].type2.data;
              int mv = 0;
              adc_cali_raw_to_voltage(cali_handle, raw_val, &mv); 
              float v = mv / 1000.0f;
              volts_array[samples_read++] = v;
              sum_volts += v;
            }
          }
        }
        
        float dc_offset = sum_volts / CHUNK_SIZE;
        float sum_sq = 0;
        for (int i = 0; i < CHUNK_SIZE; i++) {
          float centered_v = volts_array[i] - dc_offset;
          sum_sq += (centered_v * centered_v);
        }
        
        float rms = sqrt(sum_sq / CHUNK_SIZE);
        float chunk_db = (rms > 0.0001f) ? (REF_DB + 20.0f * log10(rms / MIC_SENS)) : 30.0f;

        total_db_accum += chunk_db;
        chunk_count++;

        if (chunk_db < dynamic_sustain_db) {
          silence_count++;
          silence_db_accum += chunk_db; 
        } else {
          silence_count = 0;
          silence_db_accum = 0;         
        }

        vTaskDelay(pdMS_TO_TICKS(1)); 
      }

      stop_dma_sampler(); 
      digitalWrite(LED_PIN, LOW); 

      // 4. GET TIMESTAMP & BUILD JSON
      struct tm timeinfo;
      char timestamp[30] = "Unknown Time";
      if (getLocalTime(&timeinfo, 10)) {
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &timeinfo);
      }

      // --- TRANSIENT SPIKE PROTECTION MATH ---
      int active_chunks = chunk_count - silence_count;
      float final_avg = 0.0f;
      int actual_duration_ms = 0;

      if (active_chunks > 0) {
        final_avg = (total_db_accum - silence_db_accum) / active_chunks;
        actual_duration_ms = active_chunks * 125; 
      } else {
        final_avg = total_db_accum / chunk_count;
        actual_duration_ms = chunk_count * 125;
      }

      event_counter++;
      
      char jsonBuffer[256];
      snprintf(jsonBuffer, sizeof(jsonBuffer), 
               "{\"event\":%d, \"timestamp\":\"%s\", \"dB_SPL\":%.1f, \"dur_ms\":%d}", 
               event_counter, timestamp, final_avg, actual_duration_ms);

      // 5. TRANSMIT VIA WIFI UDP 
      WiFi.mode(WIFI_STA);
      WiFi.config(local_IP, gateway, subnet); // --- INSTANT CONNECT BYPASS ---
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      unsigned long wifiStart = millis();
      
      // Still wait for connection, but it should be almost instant now
      while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 5000) {
        vTaskDelay(pdMS_TO_TICKS(50)); 
      }

      if (WiFi.status() == WL_CONNECTED) {
        
        // --- 1 PACKET ARP FLUSH ---
        udp.beginPacket(TARGET_PC_IP, UDP_PORT);
        udp.print(jsonBuffer);
        udp.endPacket();
        
        vTaskDelay(pdMS_TO_TICKS(300)); // Allow router to reply to ARP before killing radio
        
        for(int i=0; i<3; i++) {
          digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(80));
          digitalWrite(LED_PIN, LOW);  vTaskDelay(pdMS_TO_TICKS(80));
        }
      }

      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      
      if (chunk_count >= MAX_CHUNKS) {
        baseline_p2p_v = current_p2p_v; 
        vTaskDelay(pdMS_TO_TICKS(10000)); 
      }

    } else {
      baseline_p2p_v = (baseline_p2p_v * 0.95f) + (current_p2p_v * 0.05f);
    }

    // 6. DEEP POWER SAVE
    setCpuFrequencyMhz(20); 
    esp_sleep_enable_timer_wakeup(130000); 
    gpio_deep_sleep_hold_en();
    esp_light_sleep_start();
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  analogReadResolution(12);

  btStop(); // Physically kill Bluetooth baseband

  if (!init_adc_calibration()) {
    while(true) { 
      digitalWrite(LED_PIN, HIGH); delay(100); digitalWrite(LED_PIN, LOW); delay(100);
    }
  }

  dma_buf = (uint8_t*)heap_caps_malloc(CHUNK_SIZE * sizeof(adc_digi_output_data_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

  if (!dma_buf) {
    while(true) { 
      digitalWrite(LED_PIN, HIGH); delay(50); digitalWrite(LED_PIN, LOW); delay(50);
    }
  }

  initWiFiAndTime();

  xTaskCreatePinnedToCore(SentryTask, "SentryTask", 8192, NULL, 1, NULL, 1);
  vTaskDelete(NULL); 
}

void loop() {}