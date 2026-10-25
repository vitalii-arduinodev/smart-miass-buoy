#pragma once

#include <Arduino.h>

// ------------------------------------------------------------------
// Modem power management and deep sleep
// ------------------------------------------------------------------
// A7670 PWRKEY: short pulse (~100-500 ms) turns the modem on,
// a long pulse (>1.2 s, typically 2-3 s) turns it off.
void modemPowerKeyOffBegin();
void modemPowerKeyOffEnd();
void modemResetPinsForSleep();

// Release the GPIO holds latched during the previous deep sleep — call once
// at boot, before reconfiguring those pins as outputs
void releaseGpioHoldsAfterWake();

// Prepare GPIOs for deep sleep and enter sleep
void prepareSleep(uint64_t sleepUs);
