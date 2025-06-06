#define CT_ADC_PIN 36          // ESP32 ADC pin (GPIO36 = ADC1_CH0)
#define VREF 3.3            // ADC reference voltage
#define ADC_RES 4095        // 12-bit ADC
#define VOLTAGE_BIAS 1.580  // Measured midpoint voltage (bias)
#define SENSOR_SENSITIVITY 0.1  // 0.1 V/A (for SCT-013-010)
#define SAMPLE_DURATION 200  // Sampling time in milliseconds (for 10 cycles at 50Hz)

#define VAC_SENS_ADC_PIN 13


void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // ESP32 default is 12-bit
}

void loop() {
  unsigned long start_time = millis();
  float sum_squared = 0;
  int sample_count = 0;

  while ((millis() - start_time) < SAMPLE_DURATION) {
    int adc_val = analogRead(CT_ADC_PIN);
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
  //scaling Correction, by board measurement 
  rms_current *= 0.258/3.34;

  Serial.print("Irms = ");
  Serial.print(rms_current, 3);
  Serial.println(" A");


  int adc_val = analogRead(VAC_SENS_ADC_PIN); // 0 to 4095
  float voltageSense = (adc_val / 4095.0) * 3.3; // Convert to volts
  //scaling correction by board measurement
  float voltMains = 242*voltageSense/2.98;


  Serial.print("Vrms = ");
  Serial.print(voltMains, 3);
  Serial.println(" V");

  float power = voltMains * rms_current;
  Serial.print("Power = ");
  Serial.print(power, 3);
  Serial.println(" Watts");

  delay(1000); // Wait before next reading
}


