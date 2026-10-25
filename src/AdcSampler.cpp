#include "AdcSampler.h"

static uint16_t adcSamples[ADC_NUM_SAMPLES];
static uint8_t  adcIndex = 0;
static uint32_t adcLastTime = 0;
static uint32_t adcSettleTarget = 0;
static bool     adcSamplingActive = false;

// Insertion sort (replaces bubble sort)
static void insertionSort(uint16_t arr[], uint8_t len) {
    for (uint8_t i = 1; i < len; ++i) {
        uint16_t key = arr[i];
        int8_t j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j+1] = arr[j];
            --j;
        }
        arr[j+1] = key;
    }
}

static uint32_t medianFilteredSum(uint16_t arr[], uint8_t len, uint8_t discard) {
    insertionSort(arr, len);
    uint32_t sum = 0;
    for (uint8_t i = discard; i < len - discard; ++i) {
        sum += arr[i];
    }
    return sum;
}

// Call this ONCE when entering a measurement phase, not on every loop pass.
// settleDelay: time to wait before the first sample (sensor power-up).
void startAdcSampling(uint32_t settleDelay) {
    adcIndex = 0;
    adcSamplingActive = false;
    adcSettleTarget = millis() + settleDelay;
}

void processAdcSampling(uint8_t pin, bool& finished, float& resultVoltage) {
    finished = false;
    if (!adcSamplingActive) {
        // Wait until settle time passed
        if ((int32_t)(millis() - adcSettleTarget) >= 0) {
            adcSamplingActive = true;
            adcLastTime = millis();
        }
        return;
    }

    if ((int32_t)(millis() - adcLastTime) >= ADC_INTERVAL_MS) {
        // First sample is a warm-up, discard it by reading once before actual sampling
        if (adcIndex == 0) {
            analogReadMilliVolts(pin); // discard warm-up read
            adcLastTime = millis();
            return;
        }

        // analogReadMilliVolts applies factory eFuse calibration
        adcSamples[adcIndex++] = analogReadMilliVolts(pin);
        adcLastTime = millis();

        if (adcIndex >= ADC_NUM_SAMPLES) {
            uint32_t sum = medianFilteredSum(adcSamples, ADC_NUM_SAMPLES, ADC_DISCARD);
            uint8_t count = ADC_NUM_SAMPLES - 2 * ADC_DISCARD;
            resultVoltage = (float)sum / (float)count / 1000.0f;
            finished = true;
            adcSamplingActive = false;
        }
    }
}
