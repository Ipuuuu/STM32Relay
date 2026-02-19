#ifndef RECEIVER_PROTOCOL_H
#define RECEIVER_PROTOCOL_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "ECC.h"
#include "tlib.h"
#include <Servo.h>

#include "config.h"

namespace Receiver {

// Pin mapping: translates relay library pin indices to STM32duino pin numbers.
// The master's enum (STM32Relay::PB5 = 4) does NOT match STM32duino's (PB5 = 21).
// This table provides the translation.
static const uint8_t RELAY_PIN_MAP[] = {
    PB9,   // 0
    PB8,   // 1
    PB7,   // 2
    PB6,   // 3
    PB5,   // 4
    PB4,   // 5
    PB3,   // 6
    PA15,  // 7
    PA12,  // 8
    PA11,  // 9
    PA10,  // 10
    PA9,   // 11
    PA8,   // 12
    PB15,  // 13
    PB14,  // 14
    PB13,  // 15
    PB12,  // 16
    PC13,  // 17
    PC14,  // 18
    PC15,  // 19
    PA0,   // 20
    PA1,   // 21
    PA2,   // 22
    PA3,   // 23
    PA4,   // 24
    PA5,   // 25
    PA6,   // 26
    PA7,   // 27
    PB0,   // 28
    PB1,   // 29
    PB10,  // 30
    PB11,  // 31
    PB2,   // 32
    PA13,  // 33
    PA14,  // 34
};
static const uint8_t RELAY_PIN_COUNT = sizeof(RELAY_PIN_MAP) / sizeof(RELAY_PIN_MAP[0]);

inline uint8_t resolvePin(uint8_t relayPin) {
    if (relayPin < RELAY_PIN_COUNT) return RELAY_PIN_MAP[relayPin];
    return 0xFF; // Invalid pin
}


class ProtocolHandler {

public:
    enum State {
        IDLE,
        READING_PORT,
        READING_DATA_1,
        READING_DATA_2,
        EXECUTE
    };

    //reply bits
    enum REPLY_TYPE{
    REPLY_D_LOW = 0,
    REPLY_D_HIGH = 1,
    REPLY_A_VALUE = 2,
    REPLY_RETRANSMIT = 3
};

private:
    TDEV* tdev; // transmission device
    State currentState;
    Relay::Packet currentPacket;
    int expectedDataBytes;
    int receivedDataBytes;
    Servo servos[10];  // Support up to 10 servos
    uint8_t servoCount;
    struct ServoInfo {
        uint8_t pin;
        bool attached;
    } servoMap[10];

public:
    ProtocolHandler(TDEV* tdev);
    
    void begin(){tdev->begin();}
    void recover() {tdev->recover();}
    void processIncomingByte(uint8_t byte);
    void processIncomingBytes();

    void attachServo(uint8_t pin);
    void detachServo(uint8_t pin);
    
private:
    void handleIdleState(uint8_t byte);
    void handleReadingPortState(uint8_t byte);
    void handleReadingData1State(uint8_t byte);
    void handleReadingData2State(uint8_t byte);
    void executeCommand();
    
    void sendDigitalReadResponse(uint8_t pin, bool value);
    void sendAnalogReadResponse(uint8_t pin, uint16_t value);
    void sendRetransmitRequest();
    
    void resetState();
};

} // namespace Receiver

#endif