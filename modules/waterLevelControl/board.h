#ifndef BOARD_H
#define BOARD_H

#ifndef HW_V2
#define HW_V2
// Select hardware profile at compile time:
// - Define HW_V2 for new hardware
// - If HW_V2 is not defined, old hardware mapping is used

#ifdef HW_V2

// GPIO Pin Definitions (HW_V2)
#define OHT_FULL_PIN        15
#define OHT_EMPTY_PIN       04
#define UGT_NOT_EMPTY_PIN   16 //RX2
#define UGT_EMPTY_PIN       17 //TX2

#define ALARM_CLEAR_PIN     05
#define MANUAL_MODE_SWITCH  18

#define PUMP_CONTROL_PIN    12
#define PUMP_LED_PIN        02
#define BUZZER_PIN          33

// Status LEDs (HW_V2)
#define LED_ALRM_PIN        14
#define LED_PUMP_PIN        26
#define LED_PWR_PIN         27

// ADC Parameters (HW_V2)
#define VREF 3.3
#define ADC_RES 4095.0

// Current sense (HW_V2)
#define CT_SENS_PIN 36

#define VOLTAGE_BIAS 1.480
#define SENSOR_SENSITIVITY 0.1
#define SAMPLE_DURATION 200

// Dry-run thresholds (HW_V2)
#define DRY_RUN_CURRENT_THRESHOLD   0.8
#define DRY_RUN_RECOVERY_THRESHOLD  2.5

// Voltage Sense (HW_V2)
#define VAC_SENS_ADC_PIN 35
#define VAC_SCALING_FACTOR 237.0/3.1 // Correction factor based on board hw_v2 measurements

// Dry-run timing (HW_V2)
const unsigned long DRY_RUN_DELAY_MS = 0.5 * 60 * 1000;
const unsigned long DRY_RUN_BUZZ_INTERVAL = 30 * 60 * 1000;

#else

// GPIO Pin Definitions (OLD HW)
#define OHT_FULL_PIN        15
#define OHT_EMPTY_PIN       04
#define UGT_NOT_EMPTY_PIN   16
#define UGT_EMPTY_PIN       17

#define ALARM_CLEAR_PIN     05
#define MANUAL_MODE_SWITCH  18

#define PUMP_CONTROL_PIN    32
#define PUMP_LED_PIN        02
#define BUZZER_PIN          33

// Status LEDs (OLD HW)
#define LED_ALRM_PIN        27
#define LED_PUMP_PIN        26
#define LED_PWR_PIN         14

// ADC Parameters (OLD HW)
#define VREF 3.3
#define ADC_RES 4095

// Current sense (OLD HW)
#define CT_SENS_PIN 36

#define VOLTAGE_BIAS 1.580
#define SENSOR_SENSITIVITY 0.1
#define SAMPLE_DURATION 200

// Dry-run thresholds (OLD HW)
#define DRY_RUN_CURRENT_THRESHOLD   0.8
#define DRY_RUN_RECOVERY_THRESHOLD  2.5

// Voltage Sense (OLD HW)
#define VAC_SENS_ADC_PIN 13
#define VAC_SCALING_FACTOR 242/2.98 // Correction factor based on board measurements

// Dry-run timing (OLD HW)
const unsigned long DRY_RUN_DELAY_MS = 0.5 * 60 * 1000;
const unsigned long DRY_RUN_BUZZ_INTERVAL = 30 * 60 * 1000;

#endif
#endif
#endif