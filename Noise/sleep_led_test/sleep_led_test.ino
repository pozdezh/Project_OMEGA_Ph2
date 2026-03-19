#include <Arduino.h>
#include "driver/rtc_io.h" // Required for deep sleep pin control

// --- RTC MEMORY VARIABLES ---
// These survive the Deep Sleep "death" of the main CPU
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool isMonitoring = false;

// --- HARDWARE PINS ---
const gpio_num_t BUTTON_PIN = GPIO_NUM_6;
const int TFT_LIT_PIN = 13; // We will use your display backlight as a status LED for now

// Helper function to print why the ESP32 woke up
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: 
      Serial.println("Wakeup Reason: Button Pressed! (EXT0)"); 
      break;
    default: 
      Serial.printf("Wakeup Reason: Hard Reset or Power On (%d)\n", wakeup_reason); 
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500); // Give the serial monitor time to connect after wake

  // Increment and print boot count
  bootCount++;
  Serial.println("\n--- ESP32-S3 WOKE UP ---");
  Serial.println("Boot number: " + String(bootCount));

  print_wakeup_reason();

  // Initialize your TFT Backlight pin
  pinMode(TFT_LIT_PIN, OUTPUT);

  // --- LOGIC: TOGGLE STATE ON BUTTON PRESS ---
  // If we woke up because of the button, flip our system state
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    isMonitoring = !isMonitoring; 
  } else {
    // If it was a hard reset (plugging in battery), default to OFF
    isMonitoring = false; 
  }

  // --- ACTION BASED ON STATE ---
  if (isMonitoring) {
    Serial.println("System State: ACTIVE (Simulating Listening Mode...)");
    digitalWrite(TFT_LIT_PIN, HIGH); // Turn ON backlight
    delay(4000); // Stay awake for 4 seconds so you can see it
  } else {
    Serial.println("System State: SLEEPING (Shutting down...)");
    digitalWrite(TFT_LIT_PIN, LOW); // Turn OFF backlight
    delay(1000); // Brief pause before sleeping
  }

// --- PREPARE FOR DEEP SLEEP ---
  Serial.println("Configuring Deep Sleep...");

  // 1. Wakeup Trigger
  esp_sleep_enable_ext0_wakeup(BUTTON_PIN, 1);

  // 2. Lock the Button Pin (prevent false wakeups)
  rtc_gpio_pullup_dis(BUTTON_PIN);
  rtc_gpio_pulldown_en(BUTTON_PIN);

  // 3. THE BUG FIX: Lock the TFT Backlight Pin LOW
  // Hand control to the RTC, set it as output, and force it to 0V during sleep
  rtc_gpio_init((gpio_num_t)TFT_LIT_PIN); 
  rtc_gpio_set_direction((gpio_num_t)TFT_LIT_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level((gpio_num_t)TFT_LIT_PIN, 0); 

  Serial.println("Going to Deep Sleep now. Press the button to wake me up.");
  Serial.flush(); 

  // Night night.
  esp_deep_sleep_start();
}

void loop() {
  // In a true event-driven deep sleep architecture, 
  // the main loop remains completely empty.
}