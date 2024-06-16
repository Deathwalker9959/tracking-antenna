// radio.h
#ifndef RADIO_H
#define RADIO_H

#include <RH_RF95.h>
#include "helper.h"
#include "secrets.h"

// Radio constants
#define RFM95_CS 10
#define RFM95_RST 11
#define RFM95_INT 9
#define RF95_FREQ 434.0

// Create radio object
extern RH_RF95 rf95;

void initializeRadio() {
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);

    digitalWrite(RFM95_RST, LOW);
    delay(100);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);

    if (!rf95.init()) {
        Serial.println("LoRa radio init failed");
        while (1);
    }
    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        while (1);
    }

    rf95.setTxPower(20, false);
}

struct GPSData {
    int32_t lat;
    int32_t lon;
    int16_t alt;
    uint8_t sats;
    uint32_t time;

    void decryptAndLoad(const uint8_t* buf, uint8_t len, const char* psk) {
        xorEncrypt((uint8_t*)buf, len, PSK);  // Assuming xorEncrypt can be used for decryption too.
        if (len == 15) {
            memcpy(&lat, buf, 4);
            memcpy(&lon, buf + 4, 4);
            memcpy(&alt, buf + 8, 2);
            memcpy(&sats, buf + 10, 1);
            memcpy(&time, buf + 11, 4);
        }
    }
};

void sendAck() {
    const char ackMsg[] = "Data Received";
    rf95.send((uint8_t *)ackMsg, sizeof(ackMsg));
    rf95.waitPacketSent();
    // Serial.println("Sent ack.");
}

#endif // RADIO_H
