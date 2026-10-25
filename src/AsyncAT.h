#pragma once

#include <Arduino.h>

// ------------------------------------------------------------------
// Async AT command engine (non-blocking)
// ------------------------------------------------------------------
class AsyncAT {
public:
    enum class State : uint8_t {
        IDLE, SENDING, WAITING_PROMPT, SENDING_DATA,
        WAITING_RESPONSE, DRAINING, DONE
    };

    explicit AsyncAT(HardwareSerial& ser) : _serial(ser) {}

    // HardwareSerial::begin(baud, config, rxPin, txPin):
    // rxPin of the ESP32 connects to the modem TX line.
    void begin(uint32_t baud, uint8_t espRxPin, uint8_t espTxPin);

    // Send simple command and wait for successPattern (e.g. "OK").
    // When successPattern is an info line (e.g. "+CREG: 0,1"), the trailing
    // final "OK" is drained as well, so it cannot satisfy the NEXT command.
    void startCommand(const char* cmd, const char* successPattern, uint32_t timeoutMs);

    // Send data via AT+QISEND style: after sending "AT+QISEND=id,<len>"
    // wait for '>' prompt, then write raw data, then wait for final response.
    void startSendData(const char* cmd, const uint8_t* data, size_t len,
                       const char* successPattern, uint32_t timeoutMs);

    // Wait for an unsolicited line (e.g. "+QIURC: \"recv\"") without sending
    // anything and without flushing pending input.
    void startWait(const char* pattern, uint32_t timeoutMs);

    void process();

    bool isFinished() const { return _state == State::DONE; }
    bool isSuccess() const { return _result; }
    const char* lastResponse() const { return _lastResponse; }

    // True once a "+QIURC: \"recv\"" line has been seen since the last
    // startCommand/startSendData — i.e. peer data already arrived.
    bool recvUrcSeen() const { return _recvUrcSeen; }

private:
    HardwareSerial& _serial;
    State _state = State::IDLE;
    bool _withData = false;
    bool _awaitFinalOk = false;
    uint32_t _started = 0;
    uint32_t _timeout = 0;
    const char* _pattern = "OK";
    bool _result = false;
    bool _recvUrcSeen = false;
    char _line[128];
    uint16_t _lineIdx = 0;
    char _lastResponse[128] = {0};
    const uint8_t* _sendData = nullptr;
    size_t _sendLen = 0;

    static bool contains(const char* str, const char* sub);
    void startNew(const char* successPattern, uint32_t timeoutMs);
    void flushInput();
};
