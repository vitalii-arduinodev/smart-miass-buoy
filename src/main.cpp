#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Config.h"
#include "AsyncAT.h"
#include "AdcSampler.h"
#include "Measurements.h"
#include "PowerManager.h"
#include "MqttPackets.h"

// ------------------------------------------------------------------
// Global objects
// ------------------------------------------------------------------
HardwareSerial SerialAT(2); // UART2 for modem
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature tempSensor(&oneWire);
AsyncAT at(SerialAT);

// Shared measurement storage (defined here, declared in Measurements.h)
Measurements meas;

// RTC-backed variables (defined here, declared in Measurements.h)
RTC_DATA_ATTR uint32_t bootCount = 0;
RTC_DATA_ATTR uint8_t  errorCount = 0;

// ------------------------------------------------------------------
// FSM states
// ------------------------------------------------------------------
enum class SystemState : uint8_t {
    STATE_INIT,
    STATE_MEASURE_PH,
    STATE_MEASURE_TDS,
    STATE_MEASURE_OTHERS,
    STATE_MODEM_CONNECT,
    STATE_SEND_MQTT,
    STATE_MODEM_POWER_DOWN,
    STATE_SLEEP,
    STATE_ERROR
};

SystemState currentState = SystemState::STATE_INIT;
uint8_t phase = 0;
uint8_t modemAttempts = 0;
uint32_t stateEntered = 0;
uint32_t cycleStart = 0;

// ------------------------------------------------------------------
// FSM helpers
// ------------------------------------------------------------------
void gotoErrorState() {
    currentState = SystemState::STATE_ERROR;
    phase = 0;
    stateEntered = millis();
}

void checkGlobalTimeout() {
    if (currentState == SystemState::STATE_SLEEP ||
        currentState == SystemState::STATE_ERROR) return;

    if (millis() - cycleStart > GLOBAL_TIMEOUT_MS) {
        gotoErrorState();
    }
}

// Extract the <stat> field from a "+CREG: <n>[,...],<stat>" response
// (handles both the short "0,1" and the extended "\"LAC\",\"CI\",1" forms)
int parseCregStat(const char* resp) {
    if (!resp) return -1;
    const char* lastComma = strrchr(resp, ',');
    if (!lastComma) return -1;
    return atoi(lastComma + 1);
}

// ------------------------------------------------------------------
// setup()
// ------------------------------------------------------------------
void setup() {
    ++bootCount;

    Serial.begin(115200);
    Serial.printf("[buoy] boot #%lu, consecutive errors=%u\n",
                  (unsigned long)bootCount, (unsigned)errorCount);

    // Release GPIO holds latched during the previous deep sleep,
    // otherwise digitalWrite on held pins has no effect
    releaseGpioHoldsAfterWake();

    // Configure sensor power pins
    pinMode(PH_EN_PIN, OUTPUT);
    pinMode(TDS_EN_PIN, OUTPUT);
    digitalWrite(PH_EN_PIN, LOW);
    digitalWrite(TDS_EN_PIN, LOW);

    // Configure modem PWRKEY.
    // Note: GPIO12 is a strapping pin (MTDI) on the ESP32 — it must be LOW
    // at boot; verify the actual PWRKEY pin of your T-Call board revision.
    pinMode(MODEM_PWRKEY, OUTPUT);
    digitalWrite(MODEM_PWRKEY, LOW);

    // Leak sensor
    pinMode(LEAK_SENSOR_PIN, INPUT_PULLUP);

    // ADC: 11 dB attenuation covers the full 0..3.3 V range
    analogSetPinAttenuation(PH_ADC_PIN, ADC_11db);
    analogSetPinAttenuation(TDS_ADC_PIN, ADC_11db);
    analogSetPinAttenuation(TURBIDITY_ADC_PIN, ADC_11db);
    analogSetPinAttenuation(VBAT_ADC_PIN, ADC_11db);

    // Temperature sensor
    tempSensor.begin();
    tempSensor.setWaitForConversion(false);

    // Modem UART: ESP32 RX pin (26) <- modem TX, ESP32 TX pin (27) -> modem RX
    at.begin(115200, MODEM_TX_PIN, MODEM_RX_PIN);

    // Start FSM
    cycleStart = millis();
    currentState = SystemState::STATE_INIT;
    phase = 0;
    stateEntered = millis();
}

// ------------------------------------------------------------------
// loop()
// ------------------------------------------------------------------
void loop() {
    checkGlobalTimeout();

    switch (currentState) {

    // ========== INIT ==========
    case SystemState::STATE_INIT:
        currentState = SystemState::STATE_MEASURE_PH;
        phase = 0;
        stateEntered = millis();
        break;

    // ========== MEASURE PH ==========
    case SystemState::STATE_MEASURE_PH:
        if (phase == 0) {
            digitalWrite(PH_EN_PIN, HIGH);
            startAdcSampling(SENSOR_SETTLE_MS);
            phase = 1;
        } else if (phase == 1) {
            bool finished = false;
            float voltage = 0;
            processAdcSampling(PH_ADC_PIN, finished, voltage);
            if (finished) {
                meas.phValue = PH_OFFSET + (2.5f - voltage) / PH_SLOPE;
                digitalWrite(PH_EN_PIN, LOW);
                currentState = SystemState::STATE_MEASURE_TDS;
                phase = 0;
                stateEntered = millis();
            }
        }
        break;

    // ========== MEASURE TDS (raw voltage; final value after temperature) ==========
    case SystemState::STATE_MEASURE_TDS:
        if (phase == 0) {
            // Small inter-sensor pause
            if (millis() - stateEntered >= 300) {
                digitalWrite(TDS_EN_PIN, HIGH);
                startAdcSampling(SENSOR_SETTLE_MS);
                phase = 1;
            }
        } else if (phase == 1) {
            bool finished = false;
            float voltage = 0;
            processAdcSampling(TDS_ADC_PIN, finished, voltage);
            if (finished) {
                meas.tdsVoltage = voltage;
                digitalWrite(TDS_EN_PIN, LOW);
                currentState = SystemState::STATE_MEASURE_OTHERS;
                phase = 0;
                stateEntered = millis();
            }
        }
        break;

    // ========== MEASURE OTHERS ==========
    case SystemState::STATE_MEASURE_OTHERS:
        if (phase == 0) {
            tempSensor.requestTemperatures();
            stateEntered = millis();
            phase = 1;
        } else if (phase == 1) {
            if (millis() - stateEntered >= 750) {
                float t = tempSensor.getTempCByIndex(0);
                // -127 = DEVICE_DISCONNECTED_C, 85.0 = DS18B20 power-on sentinel
                meas.tempValid = !(isnan(t) || t < -50.0f || t > 84.0f);
                // Neutral 25 °C for TDS compensation when the reading is bad
                meas.tempC = meas.tempValid ? t : 25.0f;
                // Finalize TDS with temperature compensation (~2%/°C from 25 °C)
                meas.tdsValue = (meas.tdsVoltage / ADC_VREF) * TDS_COEFF *
                                (1.0f + 0.02f * (25.0f - meas.tempC));
                if (meas.tdsValue < 0.0f) meas.tdsValue = 0.0f;
                // Turbidity next: arm sampling ONCE, then only poll it
                startAdcSampling(0);
                phase = 2;
            }
        } else if (phase == 2) {
            // Turbidity (no power switch, sensor always on)
            bool finished = false;
            float voltage = 0;
            processAdcSampling(TURBIDITY_ADC_PIN, finished, voltage);
            if (finished) {
                meas.turbidityValue = (ADC_VREF - voltage) / ADC_VREF * 100.0f;
                if (meas.turbidityValue < 0.0f) meas.turbidityValue = 0.0f;
                // Battery next: arm sampling ONCE
                startAdcSampling(0);
                phase = 3;
            }
        } else if (phase == 3) {
            // Battery voltage
            bool finished = false;
            float voltage = 0;
            processAdcSampling(VBAT_ADC_PIN, finished, voltage);
            if (finished) {
                meas.vBatValue = voltage * VBAT_DIVIDER;
                phase = 4;
            }
        } else if (phase == 4) {
            meas.leakDetected = (digitalRead(LEAK_SENSOR_PIN) == LOW);
            currentState = SystemState::STATE_MODEM_CONNECT;
            phase = 0;
            stateEntered = millis();
        }
        break;

    // ========== MODEM CONNECT ==========
    case SystemState::STATE_MODEM_CONNECT:
        if (phase == 0) {
            // Power on impulse
            digitalWrite(MODEM_PWRKEY, HIGH);
            stateEntered = millis();
            modemAttempts = 0;
            phase = 1;
        } else if (phase == 1) {
            if (millis() - stateEntered >= MODEM_PWRKEY_ON_MS) {
                digitalWrite(MODEM_PWRKEY, LOW);
                stateEntered = millis();
                phase = 2;
            }
        } else if (phase == 2) {
            // Wait for modem firmware to boot
            if (millis() - stateEntered >= MODEM_BOOT_MS) {
                at.startCommand("AT", "OK", 5000);
                phase = 3;
            }
        } else if (phase == 3) {
            // "AT" with retries: right after boot the modem may not answer yet
            at.process();
            if (at.isFinished()) {
                if (at.isSuccess()) {
                    at.startCommand("ATE0", "OK", 2000); // echo off
                    phase = 4;
                } else if (++modemAttempts < MODEM_AT_RETRIES) {
                    stateEntered = millis();
                    phase = 14;
                } else {
                    gotoErrorState();
                }
            }
        } else if (phase == 14) {
            if (millis() - stateEntered >= MODEM_AT_RETRY_PAUSE_MS) {
                at.startCommand("AT", "OK", 5000);
                phase = 3;
            }
        } else if (phase == 4) {
            at.process();
            if (at.isFinished()) {
                if (!at.isSuccess()) gotoErrorState();
                else { at.startCommand("AT+CMEE=1", "OK", 2000); phase = 5; }
            }
        } else if (phase == 5) {
            at.process();
            if (at.isFinished()) {
                if (!at.isSuccess()) gotoErrorState();
                else {
                    modemAttempts = 0;
                    at.startCommand("AT+CPIN?", "+CPIN: READY", 5000);
                    phase = 6;
                }
            }
        } else if (phase == 6) {
            // SIM readiness with retries (may take a moment after boot)
            at.process();
            if (at.isFinished()) {
                if (at.isSuccess()) {
                    modemAttempts = 0;
                    at.startCommand("AT+CREG?", "+CREG:", 5000);
                    phase = 8;
                } else if (++modemAttempts < MODEM_CPIN_RETRIES) {
                    stateEntered = millis();
                    phase = 7;
                } else {
                    gotoErrorState();
                }
            }
        } else if (phase == 7) {
            if (millis() - stateEntered >= MODEM_CPIN_RETRY_PAUSE_MS) {
                at.startCommand("AT+CPIN?", "+CPIN: READY", 5000);
                phase = 6;
            }
        } else if (phase == 8) {
            // Network registration poll: stat 1 = home, 5 = roaming
            at.process();
            if (at.isFinished()) {
                if (!at.isSuccess()) {
                    gotoErrorState();
                } else {
                    int stat = parseCregStat(at.lastResponse());
                    if (stat == 1 || stat == 5) {
                        char cmd[64];
                        snprintf(cmd, sizeof(cmd), "AT+QICSGP=1,1,\"%s\"", APN);
                        at.startCommand(cmd, "OK", 5000);
                        phase = 10;
                    } else if (++modemAttempts < MODEM_CREG_RETRIES) {
                        stateEntered = millis();
                        phase = 9;
                    } else {
                        gotoErrorState();
                    }
                }
            }
        } else if (phase == 9) {
            if (millis() - stateEntered >= MODEM_CREG_PAUSE_MS) {
                at.startCommand("AT+CREG?", "+CREG:", 5000);
                phase = 8;
            }
        } else if (phase == 10) {
            at.process();
            if (at.isFinished()) {
                if (!at.isSuccess()) gotoErrorState();
                else { at.startCommand("AT+QIACT=1", "OK", 30000); phase = 11; }
            }
        } else if (phase == 11) {
            at.process();
            if (at.isFinished()) {
                if (!at.isSuccess()) gotoErrorState();
                else { at.startCommand("AT+CSQ", "+CSQ:", 5000); phase = 12; }
            }
        } else if (phase == 12) {
            at.process();
            if (at.isFinished()) {
                if (!at.isSuccess()) {
                    gotoErrorState();
                } else {
                    int rssi = -1;
                    const char* resp = at.lastResponse();
                    if (resp && strncmp(resp, "+CSQ:", 5) == 0) {
                        rssi = atoi(resp + 5);
                    }
                    if (rssi >= 0 && rssi <= 31) meas.rssiDbm = -113 + 2 * rssi;
                    else meas.rssiDbm = -127;

                    char cmd[96];
                    snprintf(cmd, sizeof(cmd), "AT+QIOPEN=1,\"TCP\",\"%s\",%d", MQTT_BROKER, MQTT_PORT);
                    at.startCommand(cmd, "+QIOPEN: 1,", 30000);
                    phase = 13;
                }
            }
        } else if (phase == 13) {
            at.process();
            if (at.isFinished()) {
                // URC "+QIOPEN: 1,<err>": err == 0 means connected
                const char* resp = at.lastResponse();
                bool openOk = at.isSuccess() && resp &&
                              strncmp(resp, "+QIOPEN: 1,", 11) == 0 && resp[11] == '0';
                if (!openOk) {
                    gotoErrorState();
                } else {
                    currentState = SystemState::STATE_SEND_MQTT;
                    phase = 0;
                    stateEntered = millis();
                }
            }
        }
        break;

    // ========== SEND MQTT ==========
    case SystemState::STATE_SEND_MQTT:
        if (phase == 0) {
            // Build MQTT CONNECT packet. static: the async engine keeps the
            // pointer and writes the data only on later loop() passes.
            static uint8_t connectPkt[64];
            uint16_t clen = buildMqttConnect(connectPkt, sizeof(connectPkt));
            if (clen == 0) { gotoErrorState(); break; }
            char cmd[48];
            snprintf(cmd, sizeof(cmd), "AT+QISEND=1,%d", clen);
            at.startSendData(cmd, connectPkt, clen, "SEND OK", 10000);
            phase = 1;
        } else if (phase == 1) {
            at.process();
            if (at.isFinished()) {
                if (!at.isSuccess()) gotoErrorState();
                else if (at.recvUrcSeen()) phase = 3; // CONNACK already arrived
                else { at.startWait("+QIURC: \"recv\"", 15000); phase = 2; }
            }
        } else if (phase == 2) {
            // MQTT 3.1.1: wait for the broker's CONNACK before publishing
            at.process();
            if (at.isFinished()) {
                if (!at.isSuccess()) gotoErrorState();
                else phase = 3;
            }
        } else if (phase == 3) {
            // Build MQTT PUBLISH packet (static — see phase 0)
            static uint8_t publishPkt[320];
            uint16_t plen = buildMqttPublish(publishPkt, sizeof(publishPkt));
            if (plen == 0) { gotoErrorState(); break; }
            char cmd[48];
            snprintf(cmd, sizeof(cmd), "AT+QISEND=1,%d", plen);
            at.startSendData(cmd, publishPkt, plen, "SEND OK", 10000);
            phase = 4;
        } else if (phase == 4) {
            at.process();
            if (at.isFinished()) {
                if (!at.isSuccess()) gotoErrorState();
                else { at.startCommand("AT+QICLOSE=1", "OK", 10000); phase = 5; }
            }
        } else if (phase == 5) {
            at.process();
            if (at.isFinished()) {
                // QICLOSE result is not checked on purpose: the cycle is done
                // and the modem is powered off next either way
                errorCount = 0;
                currentState = SystemState::STATE_MODEM_POWER_DOWN;
                phase = 0;
                stateEntered = millis();
            }
        }
        break;

    // ========== MODEM POWER DOWN ==========
    case SystemState::STATE_MODEM_POWER_DOWN:
        if (phase == 0) {
            // Graceful shutdown attempt; "OK" is firmware-agnostic, the exact
            // power-down message differs between SIM800 ("NORMAL POWER DOWN")
            // and A76xx ("POWERED DOWN") and is not waited for
            at.startCommand("AT+CPOWD=1", "OK", 3000);
            phase = 1;
        } else if (phase == 1) {
            at.process();
            if (at.isFinished()) {
                stateEntered = millis();
                phase = 2;
            }
        } else if (phase == 2) {
            // Wait a bit for modem to fully power off
            if (millis() - stateEntered >= 2000) {
                // Hardware fallback: long PWRKEY pulse guarantees power-off
                modemPowerKeyOffBegin();
                stateEntered = millis();
                phase = 3;
            }
        } else if (phase == 3) {
            if (millis() - stateEntered >= 2000) {
                modemPowerKeyOffEnd();
                currentState = SystemState::STATE_SLEEP;
                phase = 0;
                stateEntered = millis();
            }
        }
        break;

    // ========== SLEEP ==========
    case SystemState::STATE_SLEEP:
        if (phase == 0) {
            uint64_t sleepUs = (uint64_t)SLEEP_INTERVAL_MS * 1000ULL;
            prepareSleep(sleepUs);
            // Never reached
        }
        break;

    // ========== ERROR ==========
    case SystemState::STATE_ERROR:
        if (phase == 0) {
            ++errorCount;
            digitalWrite(PH_EN_PIN, LOW);
            digitalWrite(TDS_EN_PIN, LOW);

            // Try to power off modem gracefully (best-effort)
            at.startCommand("AT+CPOWD=1", "OK", 2000);
            phase = 1;
            stateEntered = millis();
        } else if (phase == 1) {
            at.process();
            if (at.isFinished()) {
                // Hardware fallback: long PWRKEY pulse guarantees power-off
                modemPowerKeyOffBegin();
                stateEntered = millis();
                phase = 2;
            }
        } else if (phase == 2) {
            if (millis() - stateEntered >= 2000) {
                modemPowerKeyOffEnd();
                // Choose sleep interval based on consecutive errors
                uint32_t sleepMs = (errorCount > 5) ? SLEEP_INTERVAL_ERROR_MS : SLEEP_INTERVAL_MS;
                uint64_t sleepUs = (uint64_t)sleepMs * 1000ULL;
                prepareSleep(sleepUs);
            }
        }
        break;
    }
}
