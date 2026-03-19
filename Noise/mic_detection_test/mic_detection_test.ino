#include <Arduino.h>
#include <math.h>

// --- HARDWARE PINS ---
const int MIC_PIN = 8;
const int TFT_LIT_PIN = 13; // We use the TFT backlight as our "Alarm LED"

// --- SETTINGS ---
const int SAMPLE_WINDOW_MS = 50; 
const float ALARM_THRESHOLD_DB = 75.0; // The threshold you requested

// --- Calibration Constants ---
const float V_REF = 3.3;             
const float ADC_MAX = 4095.0;        
const float SYSTEM_SENSITIVITY_V_PA = 0.63; 
const float REFERENCE_SPL = 94.0; 

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  
  pinMode(MIC_PIN, INPUT);
  
  // Initialize the LED pin and ensure it starts turned OFF
  pinMode(TFT_LIT_PIN, OUTPUT);
  digitalWrite(TFT_LIT_PIN, LOW); 
  
  delay(1000); // Allow mic DC offset to stabilize
}

void loop() {
  unsigned long startMillis = millis(); 
  int signalMax = 0;
  int signalMin = 4095;

  // 1. Rapidly sample the audio for 50ms
  while (millis() - startMillis < SAMPLE_WINDOW_MS) {
    int currentSample = analogRead(MIC_PIN);
    if (currentSample < 4095) { 
      if (currentSample > signalMax) signalMax = currentSample;
      if (currentSample < signalMin) signalMin = currentSample;
    }
  }

  // 2. Amplitude to RMS Voltage Math
  int peakToPeak = signalMax - signalMin;
  float volts_pp = (peakToPeak * V_REF) / ADC_MAX;
  float volts_rms = volts_pp / 2.828;

  // 3. Calculate True dB SPL
  float dB_SPL = 30.0;
  if (volts_rms > 0.001) { 
    dB_SPL = REFERENCE_SPL + 20.0 * log10(volts_rms / SYSTEM_SENSITIVITY_V_PA);
  }

  // --- THE LED TRIGGER LOGIC ---
  // If the sound is louder than 75 dB, turn the LED ON. Otherwise, keep it OFF.
  if (dB_SPL > ALARM_THRESHOLD_DB) {
    digitalWrite(TFT_LIT_PIN, HIGH);
  } else {
    digitalWrite(TFT_LIT_PIN, LOW);
  }

  // --- SERIAL PLOTTER OUTPUT ---
  Serial.print("Baseline_50dB:50,"); 
  Serial.print("Threshold_75dB:75,"); 
  Serial.print("Room_SPL:");
  Serial.println(dB_SPL);
}