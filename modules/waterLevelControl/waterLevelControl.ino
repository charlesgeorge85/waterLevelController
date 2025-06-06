#include <Arduino.h>

// GPIO Pin Definitions
#define OHT_FULL_PIN        15
#define OHT_EMPTY_PIN       02
#define UGT_EMPTY_PIN       04
#define UGT_NOT_EMPTY_PIN   16
#define ALARM_CLEAR_PIN     17
#define MANUAL_MODE_SWITCH  05

#define PUMP_CONTROL_PIN    34
#define BUZZER_PIN          33

// ADC Parameters
#define VREF 3.3            // ADC reference voltage
#define ADC_RES 4095        // 12-bit ADC

// Current sense
#define CT_SENS_PIN 36          // ESP32 ADC pin (GPIO36 = ADC1_CH0)

#define VOLTAGE_BIAS 1.580  // Measured midpoint voltage (bias)
#define SENSOR_SENSITIVITY 0.1  // 0.1 V/A (for SCT-013-010)
#define SAMPLE_DURATION 200  // Sampling time in milliseconds (for 10 cycles at 50Hz)

// Dry-run thresholds (with optional hysteresis)
#define DRY_RUN_CURRENT_THRESHOLD   2.0  // Trigger below this
#define DRY_RUN_RECOVERY_THRESHOLD  2.5  // Reset above this

// Voltage Sense
#define VAC_SENS_ADC_PIN 13


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
  analogReadResolution(12);  // ESP32 default is 12-bit
  
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

void handleDryRun(float current) {
  if (!pumpOn) return; // ✅ Only check if pump is running

  unsigned long now = millis();
  if (now - pumpStartTime < DRY_RUN_DELAY_MS) {
    // ✅ Wait 2 minutes before checking
    return;
  }

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


float voltMon()
{
 // Voltage Sampling
  int adc_val = analogRead(VAC_SENS_ADC_PIN); // 0 to 4095
  float voltageSense = (adc_val / 4095.0) * 3.3; // Convert to volts
  //scaling correction by board measurement
  float voltMains = 242*voltageSense/2.98;
  return voltMains;
}

float currentMon()
{
  // Current Sampling
  unsigned long i_sam_start_time = millis();
  float sum_squared = 0;
  int sample_count = 0;

  while ((millis() - i_sam_start_time) < SAMPLE_DURATION) {
    int adc_val = analogRead(CT_SENS_PIN);
    float voltage = (adc_val * VREF) / ADC_RES;
    float ac_component = voltage - VOLTAGE_BIAS;

    sum_squared += ac_component * ac_component;
    sample_count++;

    delayMicroseconds(200); // ~5 kHz sampling
  }

  // Calculate RMS voltage
  float mean_squared = sum_squared / sample_count;
  float rms_voltage = sqrt(mean_squared);

  // Convert to current using sensor sensitivity
  float rms_current = rms_voltage / SENSOR_SENSITIVITY;

  //scaling Correction, bu board 
  rms_current *= 0.258/3.34;

  return(rms_current);
}

void loop() {
  
  float rms_current = currentMon();
  float rms_voltage = voltMon();
  float power = rms_voltage * rms_current;
  
  /*
  Serial.print("RMS Current = ");
  Serial.print(rms_current, 3);
  Serial.println(" A");
  
  Serial.print("Vrms = ");
  Serial.print(rms_voltage, 3);
  Serial.println(" V");

  Serial.print("Power = ");
  Serial.print(power, 3);
  Serial.println(" Watts");
*/
  manualMode = digitalRead(MANUAL_MODE_SWITCH) == LOW;
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
  handleDryRun(rms_current);

  delay(1000);
}
