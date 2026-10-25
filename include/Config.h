#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- Pin mapping for LILYGO T-Call A7670 ---
#define MODEM_TX_PIN   26   // A7670 TX -> ESP32 RX2
#define MODEM_RX_PIN   27   // A7670 RX -> ESP32 TX2
#define MODEM_PWRKEY   12   // PWRKEY input (GPIO12 is a strapping pin — must be LOW at boot; verify your T-Call revision, some use GPIO4)

// ADC1 pins only (GPIO34-37 are ADC1)
#define PH_ADC_PIN         34
#define TDS_ADC_PIN        35
#define TURBIDITY_ADC_PIN  36
#define VBAT_ADC_PIN       37

// Sensor power switch GPIOs (avoid conflicts with UART/PWRKEY)
#define PH_EN_PIN    25
#define TDS_EN_PIN   33

// OneWire bus for temperature sensor
#define ONE_WIRE_PIN 15

// Leak sensor (active LOW)
#define LEAK_SENSOR_PIN 13

// --- Network / MQTT ---
#define APN              "internet"
#define MQTT_BROKER      "test.mosquitto.org"
#define MQTT_PORT        1883
#define MQTT_TOPIC       "river/buoy/1"
#define MQTT_CLIENT_ID   "river_buoy_1"

// --- Deployment / site identification ---
// Site A = river entering the city (upstream, control)
// Site B = river leaving the city (downstream, impact)
#define SITE_ID       "A"          // change to "B" when deployed downstream
#define SITE_LAT      55.1600f     // <-- replace with real GPS coordinates of site
#define SITE_LON      61.4000f     // <-- replace with real GPS coordinates of site

// --- Sleep intervals ---
#define SLEEP_INTERVAL_MS        (30UL * 60UL * 1000UL)      // 30 min
#define SLEEP_INTERVAL_ERROR_MS  (2UL * 60UL * 60UL * 1000UL) // 2 hours
// Must exceed the sum of all modem timeouts with retries (worst case ~260 s)
#define GLOBAL_TIMEOUT_MS        300000UL

// --- Modem timing / retries ---
#define MODEM_PWRKEY_ON_MS        1000UL  // A76xx PWRKEY on-pulse (>= 1 s)
#define MODEM_BOOT_MS             5000UL  // firmware boot before the first AT
#define MODEM_AT_RETRIES          3
#define MODEM_AT_RETRY_PAUSE_MS   1000UL
#define MODEM_CPIN_RETRIES        3
#define MODEM_CPIN_RETRY_PAUSE_MS 1000UL
#define MODEM_CREG_RETRIES        10      // network registration polling attempts
#define MODEM_CREG_PAUSE_MS       2000UL

// --- ADC / calibration ---
#define ADC_VREF         3.3f
#define ADC_MAX          4095.0f
#define VBAT_DIVIDER     2.0f
#define PH_OFFSET        7.0f
#define PH_SLOPE         0.18f
#define TDS_COEFF        1000.0f

// --- Sampling settings ---
#define ADC_NUM_SAMPLES      10
#define ADC_DISCARD          2
#define ADC_INTERVAL_MS      20
#define SENSOR_SETTLE_MS     500

#endif // CONFIG_H