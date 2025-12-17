#ifndef RECEIVER_PROTOCOL_H
#define RECEIVER_PROTOCOL_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "ECC.h"
#include <Servo.h>

namespace Receiver {

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
    HardwareSerial* uart_port;
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
    ProtocolHandler(HardwareSerial* port);
    
    void begin(uint32_t baud);
    void processIncomingByte(uint8_t byte);

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