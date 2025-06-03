#include <Arduino.h>

// GPIO Pin Definitions
#define OHT_FULL_PIN        15
#define OHT_EMPTY_PIN       02
#define UGT_EMPTY_PIN       04
#define UGT_NOT_EMPTY_PIN   16
#define PUMP_CONTROL_PIN    34
#define CURRENT_SENSOR_PIN  33
#define BUZZER_PIN          33
#define ALARM_CLEAR_PIN     27
#define MANUAL_MODE_SWITCH  14

// Dry-run thresholds (with optional hysteresis)
#define DRY_RUN_CURRENT_THRESHOLD   2.0  // Trigger below this
#define DRY_RUN_RECOVERY_THRESHOLD  2.5  // Reset above this

// Dry-run delay before checking (after pump ON)
const unsigned long DRY_RUN_DELAY_MS = 2 * 60 * 1000; // 2 minutes
const unsigned long DRY_RUN_BUZZ_INTERVAL = 30 * 60 * 1000; // 30 mins

// System States
bool pumpOn = false;
bool manualMode = false;
bool dryRunDetected = false;
bool ugtPreviouslyEmpty = false;

unsigned long lastDryRunBuzzTime = 0;
unsigned long pumpStartTime = 0;

void setup() {
  Serial.begin(115200);

  // Sensor Inputs
  pinMode(OHT_FULL_PIN, INPUT);
  pinMode(OHT_EMPTY_PIN, INPUT);
  pinMode(UGT_EMPTY_PIN, INPUT);
  pinMode(UGT_NOT_EMPTY_PIN, INPUT);

  // Control Outputs
  pinMode(PUMP_CONTROL_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // User Inputs
  pinMode(ALARM_CLEAR_PIN, INPUT_PULLUP);
  pinMode(MANUAL_MODE_SWITCH, INPUT_PULLUP);

  digitalWrite(PUMP_CONTROL_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
}

bool readSensor(int pin) {
  return digitalRead(pin) == LOW; // Assume LOW means "active"
}

float readCurrent() {
  int adcValue = analogRead(CURRENT_SENSOR_PIN);
  float current = ((float)adcValue / 4095.0) * 5.0; // Simulated conversion
  return current;
}

void activatePump() {
  if (!pumpOn) {
    digitalWrite(PUMP_CONTROL_PIN, HIGH);
    pumpOn = true;
    pumpStartTime = millis();
    Serial.println("Pump ON");
  }
}

void deactivatePump() {
  if (pumpOn) {
    digitalWrite(PUMP_CONTROL_PIN, LOW);
    pumpOn = false;
    Serial.println("Pump OFF");
  }
}

void handleDryRun() {
  if (!pumpOn) return; // ✅ Only check if pump is running

  unsigned long now = millis();
  if (now - pumpStartTime < DRY_RUN_DELAY_MS) {
    // ✅ Wait 2 minutes before checking
    return;
  }

  float current = readCurrent();

  if (dryRunDetected) {
    if (current > DRY_RUN_RECOVERY_THRESHOLD) {
      dryRunDetected = false;
      Serial.println("Dry run cleared.");
    }
  } else {
    if (current < DRY_RUN_CURRENT_THRESHOLD) {
      dryRunDetected = true;
      deactivatePump();
      Serial.println("Dry run detected.");
    }
  }

  if (dryRunDetected) {
    if (now - lastDryRunBuzzTime > DRY_RUN_BUZZ_INTERVAL) {
      tone(BUZZER_PIN, 1000, 2000); // 2 sec beep
      lastDryRunBuzzTime = now;
    }

    if (digitalRead(ALARM_CLEAR_PIN) == LOW) {
      dryRunDetected = false;
      lastDryRunBuzzTime = 0;
      Serial.println("Dry-run alarm cleared.");
    }
  }
}

void loop() {
  manualMode = digitalRead(MANUAL_MODE_SWITCH) == LOW;
Serial.println("Manual Mode Active");
  if (manualMode) {
    Serial.println("Manual Mode Active");
    // Optionally handle pump ON/OFF via buttons here
  } else {
    // Read tank sensor states
    bool ohtEmpty = readSensor(OHT_EMPTY_PIN);
    bool ohtFull = readSensor(OHT_FULL_PIN);
    bool ugtEmpty = readSensor(UGT_EMPTY_PIN);
    bool ugtNotEmpty = readSensor(UGT_NOT_EMPTY_PIN);

    // Track UGT history
    if (ugtEmpty) {
      ugtPreviouslyEmpty = true;
      deactivatePump(); // Safety: prevent dry run
    }

    if (ugtPreviouslyEmpty && ugtNotEmpty) {
      ugtPreviouslyEmpty = false; // Reset lock
    }

    // Normal operation
    if (!manualMode && !dryRunDetected && !ugtPreviouslyEmpty) {
      if (ohtEmpty && !ugtEmpty && !ohtFull) {
        activatePump();
      } else {
        deactivatePump();
      }
    } else {
      deactivatePump();
    }
  }

  // Dry run check (only if pump is ON & delay passed)
  handleDryRun();

  delay(1000);
}
