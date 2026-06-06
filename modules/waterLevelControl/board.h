#ifndef BOARD_H
#define BOARD_H

// GPIO Pin Definitions
#define OHT_FULL_PIN        15
#define OHT_EMPTY_PIN       04
#define UGT_NOT_EMPTY_PIN   16
#define UGT_EMPTY_PIN       17

#define ALARM_CLEAR_PIN     05
#define MANUAL_MODE_SWITCH  18

#define PUMP_CONTROL_PIN    32
#define PUMP_LED_PIN        02
#define BUZZER_PIN          33

// Status LEDs
#define LED_ALRM_PIN        27
#define LED_PUMP_PIN        26
#define LED_PWR_PIN         14

// ADC Parameters
#define VREF 3.3
#define ADC_RES 4095

// Current sense
#define CT_SENS_PIN 36

#define VOLTAGE_BIAS 1.580
#define SENSOR_SENSITIVITY 0.1
#define SAMPLE_DURATION 200

// Dry-run thresholds
#define DRY_RUN_CURRENT_THRESHOLD   0.8
#define DRY_RUN_RECOVERY_THRESHOLD  2.5

// Voltage Sense
#define VAC_SENS_ADC_PIN 13

// Dry-run delay before checking (after pump ON)
const unsigned long DRY_RUN_DELAY_MS = 0.5 * 60 * 1000;
const unsigned long DRY_RUN_BUZZ_INTERVAL = 30 * 60 * 1000;

#endif