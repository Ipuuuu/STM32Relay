#ifndef STM32RELAY_H
#define STM32RELAY_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "Pin.h"
#include "ECC.h"

#include "tlib.h"
#include "config.h"

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
void buildPacket(Packet& packet, CommandByte::COMMAND_TYPE cmd, uint8_t pin, uint8_t singleByteValue);  



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

private:
    TDEV *tdev; // transmission device pointer

public:
    // STM32Relay constructor
    // @param tdev : transmission device pointer
    explicit STM32Relay(TDEV *tdev);

    STM32Relay(const STM32Relay&) = delete; // delete copy constructor
    STM32Relay& operator=(const STM32Relay&) = delete; // delete copy assignment

    STM32Relay(STM32Relay&&);
    STM32Relay& operator=(STM32Relay&&);

    inline TDEV *getTDEV() const{ return tdev; }
    inline TDEV *getTDEV(){ return tdev; }

    // low-level commands
    inline void begin() {tdev->begin();}
    inline void sendByte(uint8_t byte, uint8_t addr = 0x00);
    inline void sendBytes(const uint8_t *bytes, size_t length, uint8_t addr = 0x00);
    inline void recvByte(uint8_t addr = 0x00) {tdev->recvByte(addr);}
    inline void setTimeout(uint32_t timeout) {tdev->setTimeout(timeout);}
    

    STM32Relay& sendPacket(const Packet& packet, uint8_t addr = 0x00);
    bool recvPacket(Packet& packet, int expectedBytes, uint8_t addr = 0x00);

    //high-level commands
    STM32Relay&  digitalWrite(uint8_t pin, uint8_t value, uint8_t addr = 0x00);
    bool digitalRead(uint8_t pin, uint8_t addr = 0x00);
    STM32Relay&  analogWrite(uint8_t pin, uint8_t value, uint8_t addr = 0x00);
    int analogRead(uint8_t pin, uint8_t addr = 0x00);
    STM32Relay&  writePPM(uint8_t pin, uint32_t microseconds, uint8_t addr = 0x00);
    STM32Relay&  pinMode(uint8_t pin, uint8_t mode, uint8_t addr = 0x00);

};



// Global relay object needs to be rechecked and reimplemented
// because we need to pass TDEV pointer to the constructor, and we want to avoid static initialization order issues.
// One possible solution is to use a singleton pattern or a factory function to create and access the global relay object.

// extern STM32Relay relay; //global relay object

// //global function wrappers
// inline void begin(){
//     relay.getTDEV()->begin();
// }
// inline void sendByte(uint8_t byte, uint8_t addr = 0x00){
//     relay.getTDEV()->sendByte(byte, addr);
// }
// inline uint8_t recvByte(uint8_t addr = 0x00){
//     return relay.getTDEV()->recvByte(addr);
// }
// inline void digitalWrite(uint8_t pin, uint8_t value, uint8_t addr = 0x00){
//     relay.digitalWrite(pin, value, addr);
// }
// inline bool digitalRead(uint8_t pin, uint8_t addr = 0x00){
//     return relay.digitalRead(pin, addr);
// }
// inline void analogWrite(uint8_t pin, uint8_t value, uint8_t addr = 0x00){
//     relay.analogWrite(pin, value, addr);
// }
// inline int analogRead(uint8_t pin, uint8_t addr = 0x00){
//     return relay.analogRead(pin, addr);
// }
// inline void writePPM(uint8_t pin, uint32_t microseconds, uint8_t addr = 0x00){
//     relay.writePPM(pin, microseconds, addr);
// }
// inline void setTimeout(uint32_t timeout){
//     relay.getTDEV()->setTimeout(timeout);

// }

}


#endif

