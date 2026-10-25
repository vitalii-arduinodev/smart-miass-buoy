#pragma once

#include <Arduino.h>

// Shared measurement storage, filled by the FSM and read by the
// MQTT packet builders.
struct Measurements {
    float tempC = 20.0f;
    bool  tempValid = false; // false when the DS18B20 is missing or misread
    float phValue = 0.0f;
    float turbidityValue = 0.0f;
    float tdsVoltage = 0.0f; // raw TDS voltage, finalized after temperature is known
    float tdsValue = 0.0f;
    float vBatValue = 0.0f;
    int   rssiDbm = -127;
    bool  leakDetected = false;
};

extern Measurements meas;

// RTC-backed counters shared between main FSM and MQTT builders
extern RTC_DATA_ATTR uint32_t bootCount;
extern RTC_DATA_ATTR uint8_t  errorCount;
