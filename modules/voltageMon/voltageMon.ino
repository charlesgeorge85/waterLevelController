#define VAC_SENS_ADC_PIN 13

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // ESP32 ADC is 12-bit
}

void loop() {
  int adc_val = analogRead(VAC_SENS_ADC_PIN); // 0 to 4095
  float voltageSense = (adc_val / 4095.0) * 3.3; // Convert to volts
  float voltageMains = 242*voltageSense/2.98;

  Serial.println(voltageMains);

  delay(100);  // Sample every 100 ms
}