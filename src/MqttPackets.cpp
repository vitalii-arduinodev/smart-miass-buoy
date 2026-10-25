#include "MqttPackets.h"
#include "Config.h"
#include "Measurements.h"
#include <ArduinoJson.h>

uint16_t buildMqttConnect(uint8_t* buf, uint16_t maxLen) {
    uint16_t i = 0;
    buf[i++] = 0x10; // CONNECT packet
    uint16_t lenPos = i;
    buf[i++] = 0x00; // Remaining Length (will fix later)

    // Protocol name
    buf[i++] = 0x00; buf[i++] = 0x04;
    buf[i++] = 'M'; buf[i++] = 'Q'; buf[i++] = 'T'; buf[i++] = 'T';
    // Protocol level
    buf[i++] = 0x04;
    // Connect flags: Clean Session
    buf[i++] = 0x02;
    // Keep alive: 60 seconds
    buf[i++] = 0x00; buf[i++] = 0x3C;

    // Client ID
    const char* clientId = MQTT_CLIENT_ID;
    uint16_t idLen = strlen(clientId);
    if ((uint16_t)(i + 2 + idLen + 1) > maxLen) return 0; // does not fit
    buf[i++] = (uint8_t)(idLen >> 8);
    buf[i++] = (uint8_t)(idLen & 0xFF);
    memcpy(&buf[i], clientId, idLen);
    i += idLen;

    uint16_t len = i - (lenPos + 1);
    buf[lenPos] = len;

    return i;
}

uint16_t buildMqttPublish(uint8_t* buf, uint16_t maxLen) {
    const char* topic = MQTT_TOPIC;
    uint16_t topicLen = strlen(topic);

    JsonDocument doc;
    doc["boot"] = bootCount;
    doc["uptime_s"] = millis() / 1000UL; // seconds since reset (monotonic)
    doc["site"] = SITE_ID;               // "A" upstream / "B" downstream
    doc["lat"] = SITE_LAT;               // GPS of this deployment point
    doc["lon"] = SITE_LON;
    if (meas.tempValid) doc["temp"] = meas.tempC;
    else doc["temp"] = nullptr;   // sensor missing or bad reading
    doc["temp_ok"] = meas.tempValid;
    doc["ph"] = meas.phValue;
    doc["turbidity"] = meas.turbidityValue;
    doc["tds"] = static_cast<int>(meas.tdsValue);
    doc["v_bat"] = meas.vBatValue;
    doc["leak"] = meas.leakDetected;
    doc["rssi"] = meas.rssiDbm;
    doc["err"] = errorCount;

    char payload[256];
    size_t payloadLen = serializeJson(doc, payload, sizeof(payload));
    if (payloadLen == 0) return 0;

    // MQTT Remaining Length is variable-length (1..4 bytes). The payload now
    // can exceed 127 bytes, so encode it properly instead of assuming 1 byte.
    uint32_t remLen = 2 + topicLen + (uint32_t)payloadLen; // topic len + topic + payload
    uint16_t remBytes = 0;
    for (uint32_t t = remLen; t > 0; t >>= 7) ++remBytes;

    uint16_t total = 1 + remBytes + 2 + topicLen + payloadLen;
    if (total > maxLen) return 0;

    uint16_t i = 0;
    buf[i++] = 0x30; // PUBLISH packet, QoS 0

    // Encode remaining length (1..2 bytes for our payload sizes)
    uint32_t t = remLen;
    do {
        uint8_t b = (uint8_t)(t & 0x7F);
        t >>= 7;
        if (t > 0) b |= 0x80; // continuation bit set
        buf[i++] = b;
    } while (t > 0);

    buf[i++] = (uint8_t)(topicLen >> 8);
    buf[i++] = (uint8_t)(topicLen & 0xFF);
    memcpy(&buf[i], topic, topicLen);
    i += topicLen;

    memcpy(&buf[i], payload, payloadLen);
    i += payloadLen;

    return i;
}
