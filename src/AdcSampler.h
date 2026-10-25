#pragma once

#include <Arduino.h>
#include "Config.h"

// ------------------------------------------------------------------
// Non-blocking ADC sampling with median filtering
// ------------------------------------------------------------------
void startAdcSampling(uint32_t settleDelay);

// Poll from the FSM loop; sets finished=true and resultVoltage (volts)
// once ADC_NUM_SAMPLES have been collected and filtered.
void processAdcSampling(uint8_t pin, bool& finished, float& resultVoltage);
