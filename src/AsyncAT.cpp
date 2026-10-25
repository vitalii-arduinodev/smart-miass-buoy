#include "AsyncAT.h"

// Time allowed for the trailing "OK" after a matched info line
static const uint32_t DRAIN_TIMEOUT_MS = 500;

void AsyncAT::begin(uint32_t baud, uint8_t espRxPin, uint8_t espTxPin) {
    _serial.begin(baud, SERIAL_8N1, espRxPin, espTxPin);
}

void AsyncAT::startNew(const char* successPattern, uint32_t timeoutMs) {
    _started = millis();
    _timeout = timeoutMs;
    _pattern = successPattern;
    _result = false;
    _recvUrcSeen = false;
    _lineIdx = 0;
}

void AsyncAT::flushInput() {
    // Discard stale bytes (late tails of the previous response) so they
    // cannot falsely satisfy this command.
    while (_serial.available()) _serial.read();
}

void AsyncAT::startCommand(const char* cmd, const char* successPattern, uint32_t timeoutMs) {
    flushInput();
    startNew(successPattern, timeoutMs);
    _withData = false;
    // Responses like "+CREG: 0,1" are followed by a final "OK"; patterns that
    // are/contain "OK" are final by themselves and must not drain.
    _awaitFinalOk = !contains(successPattern, "OK");
    _state = State::SENDING;
    _serial.println(cmd);
}

void AsyncAT::startSendData(const char* cmd, const uint8_t* data, size_t len,
                            const char* successPattern, uint32_t timeoutMs) {
    flushInput();
    startNew(successPattern, timeoutMs);
    _withData = true;
    _awaitFinalOk = false; // final response ("SEND OK") is the pattern itself
    _sendData = data;
    _sendLen = len;
    _state = State::SENDING;
    _serial.println(cmd);
}

void AsyncAT::startWait(const char* pattern, uint32_t timeoutMs) {
    // No flush and no _recvUrcSeen reset: the awaited line may already be
    // pending in the input, or already consumed by the previous command.
    _started = millis();
    _timeout = timeoutMs;
    _pattern = pattern;
    _result = false;
    _lineIdx = 0;
    _withData = false;
    _awaitFinalOk = false;
    _state = State::WAITING_RESPONSE;
}

void AsyncAT::process() {
    if (_state == State::IDLE) return;

    if (_state == State::SENDING) {
        // Commands with a data payload must first wait for the '>' prompt.
        _state = _withData ? State::WAITING_PROMPT : State::WAITING_RESPONSE;
        _started = millis();
    }
    else if (_state == State::WAITING_PROMPT) {
        while (_serial.available()) {
            char c = _serial.read();
            if (c == 0x3E) { // '>'
                _state = State::SENDING_DATA;
                _serial.write(_sendData, _sendLen);
                _started = millis();
                return;
            }
            if (c == '\n') {
                _line[_lineIdx] = '\0';
                if (contains(_line, "ERROR")) {
                    _result = false;
                    _state = State::DONE;
                    return;
                }
                _lineIdx = 0;
            } else if (_lineIdx < sizeof(_line)-1) {
                _line[_lineIdx++] = c;
            }
        }
        if (millis() - _started > _timeout) {
            _result = false;
            _state = State::DONE;
        }
    }
    else if (_state == State::SENDING_DATA) {
        // Data already sent, now wait for final response ("SEND OK").
        _state = State::WAITING_RESPONSE;
        _started = millis();
    }
    else if (_state == State::WAITING_RESPONSE) {
        while (_serial.available()) {
            char c = _serial.read();
            if (c == '\n') {
                _line[_lineIdx] = '\0';
                // Save the last meaningful line
                if (_lineIdx > 0) {
                    strncpy(_lastResponse, _line, sizeof(_lastResponse)-1);
                    _lastResponse[sizeof(_lastResponse)-1] = '\0';
                }
                if (contains(_line, "+QIURC: \"recv\"")) _recvUrcSeen = true;
                if (contains(_line, "+QIURC: \"closed\"")) {
                    // Connection dropped — nothing here can succeed anymore
                    _result = false;
                    _state = State::DONE;
                    return;
                }
                if (contains(_line, _pattern)) {
                    _result = true;
                    if (_awaitFinalOk) {
                        _lineIdx = 0;
                        _started = millis();
                        _state = State::DRAINING;
                    } else {
                        _state = State::DONE;
                    }
                    return;
                }
                if (contains(_line, "ERROR")) {
                    _result = false;
                    _state = State::DONE;
                    return;
                }
                _lineIdx = 0;
            } else if (_lineIdx < sizeof(_line)-1) {
                _line[_lineIdx++] = c;
            }
        }
        if (millis() - _started > _timeout) {
            _result = false;
            _state = State::DONE;
        }
    }
    else if (_state == State::DRAINING) {
        // Consume the trailing "OK" of an info-line response.
        while (_serial.available()) {
            char c = _serial.read();
            if (c == '\n') {
                _line[_lineIdx] = '\0';
                if (contains(_line, "OK")) {
                    _state = State::DONE;
                    return;
                }
                if (contains(_line, "ERROR")) {
                    _result = false;
                    _state = State::DONE;
                    return;
                }
                _lineIdx = 0;
            } else if (_lineIdx < sizeof(_line)-1) {
                _line[_lineIdx++] = c;
            }
        }
        // No "OK" ever came — keep the matched result
        if (millis() - _started > DRAIN_TIMEOUT_MS) {
            _state = State::DONE;
        }
    }
}

bool AsyncAT::contains(const char* str, const char* sub) {
    return strstr(str, sub) != nullptr;
}
