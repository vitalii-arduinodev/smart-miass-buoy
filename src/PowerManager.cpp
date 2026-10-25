#include "PowerManager.h"
#include "Config.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"

void releaseGpioHoldsAfterWake() {
    gpio_hold_dis(GPIO_NUM_25);
    gpio_hold_dis(GPIO_NUM_33);
    gpio_hold_dis(GPIO_NUM_12);
    gpio_deep_sleep_hold_dis();
}

void modemPowerKeyOffBegin() {
    digitalWrite(MODEM_PWRKEY, HIGH);
}

void modemPowerKeyOffEnd() {
    digitalWrite(MODEM_PWRKEY, LOW);
}

void modemResetPinsForSleep() {
    pinMode(MODEM_TX_PIN, INPUT);
    pinMode(MODEM_RX_PIN, INPUT);
    pinMode(MODEM_PWRKEY, INPUT_PULLDOWN); // pulldown to keep PWRKEY low
}

void prepareSleep(uint64_t sleepUs) {
    // Ensure power switches are off
    digitalWrite(PH_EN_PIN, LOW);
    digitalWrite(TDS_EN_PIN, LOW);

    // Put modem UART lines into high-impedance
    modemResetPinsForSleep();

    // Hold critical GPIO states to avoid leakage during sleep
    gpio_hold_en(GPIO_NUM_25);
    gpio_hold_en(GPIO_NUM_33);
    gpio_hold_en(GPIO_NUM_12);
    gpio_deep_sleep_hold_en();

    esp_sleep_enable_timer_wakeup(sleepUs);
    esp_deep_sleep_start();
}
