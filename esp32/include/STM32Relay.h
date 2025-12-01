#ifndef STM32RELAY_H
#define STM32RELAY_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "Pin.h"
#include "ECC.h"

#define SYNC_BIT 0b10000000
#define PARITY_BIT 0b01000000

// namespace
namespace Relay{

//reply bits
enum REPLY_TYPE{
    REPLY_D_LOW = 0,
    REPLY_D_HIGH = 1,
    REPLY_A_VALUE = 2,
    REPLY_RETRANSMIT = 3
};

void buildPacket(Packet& packet, CommandByte::COMMAND_TYPE cmd, uint8_t pin);
void buildPacket(Packet& packet, CommandByte::COMMAND_TYPE cmd, uint8_t pin, uint16_t value);


class STM32Relay{
public:
    typedef enum {
    // Left side pins
    PB9   = 0,
    PB8   = 1,
    PB7   = 2,
    PB6   = 3,
    PB5   = 4,
    PB4   = 5,
    PB3   = 6,
    PA15  = 7,
    PA12  = 8,   // USB DP
    PA11  = 9,   // USB DM
    PA10  = 10,
    PA9   = 11,
    PA8   = 12,
    PB15  = 13,
    PB14  = 14,
    PB13  = 15,
    PB12  = 16,  // LED Blackpill

    // Right side pins
    PC13  = 17,  // LED Bluepill
    PC14  = 18,
    PC15  = 19,

    // Analog pins (Arduino aliases)
    PA0   = 20,  // PIN_A0
    PA1   = 21,  // PIN_A1
    PA2   = 22,  // PIN_A2
    PA3   = 23,  // PIN_A3
    PA4   = 24,  // PIN_A4
    PA5   = 25,  // PIN_A5
    PA6   = 26,  // PIN_A6
    PA7   = 27,  // PIN_A7
    PB0   = 28,  // PIN_A8
    PB1   = 29,  // PIN_A9

    PB10  = 30,
    PB11  = 31,

    // Other pins
    PB2   = 32,  // BOOT1
    PA13  = 33,  // SWDIO
    PA14  = 34,  // SWCLK

    // Alternate pin functions (ALT1, ALT2)
    PA0_ALT1  = (PA0  | (1 << 8)),
    PA1_ALT1  = (PA1  | (1 << 8)),
    PA2_ALT1  = (PA2  | (1 << 8)),
    PA3_ALT1  = (PA3  | (1 << 8)),
    PA4_ALT1  = (PA4  | (1 << 8)),
    PA5_ALT1  = (PA5  | (1 << 8)),
    PA6_ALT1  = (PA6  | (1 << 8)),
    PA7_ALT1  = (PA7  | (1 << 8)),
    PA8_ALT1  = (PA8  | (1 << 8)),
    PA9_ALT1  = (PA9  | (1 << 8)),
    PA10_ALT1 = (PA10 | (1 << 8)),
    PA11_ALT1 = (PA11 | (1 << 8)),
    PA15_ALT1 = (PA15 | (1 << 8)),
    PB0_ALT1  = (PB0  | (1 << 8)),
    PB0_ALT2  = (PB0  | (2 << 8)),
    PB1_ALT1  = (PB1  | (1 << 8)),
    PB1_ALT2  = (PB1  | (2 << 8)),
    PB3_ALT1  = (PB3  | (1 << 8)),
    PB10_ALT1 = (PB10 | (1 << 8)),
    PB11_ALT1 = (PB11 | (1 << 8)),
    PB13_ALT1 = (PB13 | (1 << 8)),
    PB14_ALT1 = (PB14 | (1 << 8)),

    // Analog aliases for Arduino compatibility
    PIN_A0 = PA0,
    PIN_A1 = PA1,
    PIN_A2 = PA2,
    PIN_A3 = PA3,
    PIN_A4 = PA4,
    PIN_A5 = PA5,
    PIN_A6 = PA6,
    PIN_A7 = PA7,
    PIN_A8 = PB0,
    PIN_A9 = PB1,

    // Counts
    STM_NUM_DIGITAL_PINS  = 35,
    STM_NUM_ANALOG_INPUTS = 10
    } pinName;

    enum commType{
        UART,
        I2C
    };
private:
    HardwareSerial *uart_port; //(e.g Serial1, serial2)
    uint8_t txPin, rxPin;
    commType comm_Type; //UART or I2C

public:
    STM32Relay(commType type, uint8_t rx, uint8_t tx);
    
    // low-level comm
    STM32Relay&  begin(int32_t baud);
    STM32Relay&  sendByte(uint8_t byte);
    STM32Relay& sendPacket(const Packet& packet); 
    uint8_t recvByte(uint32_t timeout);
    bool recvPacket(Packet& packet, uint32_t timeout, int expectedBytes);

    //high-level commands
    STM32Relay&  digitalWrite(uint8_t pin, uint8_t value);
    bool digitalRead(uint8_t pin) ;
    STM32Relay&  analogWrite(uint8_t pin, uint8_t value);
    int analogRead(uint8_t pin) ;
    STM32Relay&  writePPM(uint8_t pin, uint32_t microseconds);
    STM32Relay&  pinMode(uint8_t pin, uint8_t value);

};

extern STM32Relay relay; //global relay object

//functions
inline void begin(int32_t baud){
    relay.begin(baud);
}
inline void sendByte(uint8_t byte){
    relay.sendByte(byte);
}
inline uint8_t recvByte(uint32_t timeout){
    return relay.recvByte(timeout);
}
inline void digitalWrite(uint8_t pin, uint8_t value){
    relay.digitalWrite(pin, value);
}
inline bool digitalRead(uint8_t pin){
    return relay.digitalRead(pin);
}
inline void analogWrite(uint8_t pin, uint8_t value){
    relay.analogWrite(pin, value);
}
inline int analogRead(uint8_t pin){
    return relay.analogRead(pin);
}
inline void writePPM(uint8_t pin, uint32_t microseconds){
    relay.writePPM(pin, microseconds);
}

};


#endif