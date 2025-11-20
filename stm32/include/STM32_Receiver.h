#ifndef STM32_RECEIVER_H
#define STM32_RECEIVER_H

#include <Arduino.h>
#include <HardwareSerial.h>

#define SYNC_BIT 0b10000000
#define PARITY_BIT 0b01000000

// Command bits
enum COMMAND_TYPE {
    CMD_D_W_LOW = 0,
    CMD_D_W_HIGH = 1,
    CMD_D_R = 2,
    CMD_A_W = 3,
    CMD_A_R = 4,
    CMD_SET_PPM = 5,
    CMD_SET_PIN_MODE_INPUT = 6,
    CMD_SET_PIN_MODE_OUTPUT = 7
};

// Reply bits
enum REPLY_TYPE {
    REPLY_D_LOW = 0,
    REPLY_D_HIGH = 1,
    REPLY_A_VALUE = 2,
    REPLY_RETRANSMIT = 3
};

//stm32 state
enum State{
    IDLE, //waiting for cmd
    READING_PIN, //reading pin byte
    READING_VALUE_1, //reading value byte3
    READING_VALUE_2, //reading value byte4
    EXECUTE //execute and respond
};

extern HardwareSerial  Serial1;
extern uint8_t currentState;
extern uint8_t commandType;
extern uint8_t pinNumber;
extern uint16_t value;
extern uint8_t byte3, byte4;


uint8_t calculateParity(int data, uint8_t numBits); // (returns 1 if odd, 0 if even)
uint8_t calcOddParity(uint8_t data, uint8_t numBits); //(returns 1 if odd number of 1s, 0 if even)
uint8_t calcEvenParity(uint8_t data, uint8_t numBits); //(returns 0 if odd number of 1s, 1 if even)
bool verifyParity(uint8_t byte);
uint8_t buildPinByte(uint8_t pinNumber);
void buildValueBytes(uint16_t value, uint8_t &byte3, uint8_t &byte4);

void handleIdleState(uint8_t byte);
void handleReadingPinState(uint8_t byte);
void handleReadingValue1State(uint8_t byte);
void handleReadingValue2State(uint8_t byte);
void executeCommand();
void sendDigitalReadResponse(uint8_t pin, uint16_t value);
void sendAnalogReadResponse(uint8_t pin, uint16_t value);
void sendRetransmitRequest();

// ECC INTERMEDIATE BYTE GENERATION

/**
 * Generate intermediate byte representation for ECC calculation
 * Bit 0: odd parity of data bits
 * Bit 1: even parity of data bits
 * Bits 2-7: data bits (including original parity bit if present)
 */
uint8_t generateIntermediateByte(uint8_t originalByte, bool isFirstByte);

//Generate all intermediate bytes from original packet
void generateIntermediateBytes(uint8_t* original, uint8_t* intermediate, uint8_t numBytes);

// ECC BIT CALCULATION
uint8_t calculateP1(uint8_t* intermediate, uint8_t numBytes); //parity of bits [0,1,2,3] from all bytes
uint8_t calculateP2(uint8_t* intermediate, uint8_t numBytes); //parity of bits [0,1,4,5] from all bytes
uint8_t calculateP3(uint8_t* intermediate, uint8_t numBytes); // parity of bits [0,2,4,6] from all bytes
uint8_t calculateECCBits(uint8_t* intermediate, uint8_t numBytes); // alculate all three ECC bits and return as 3-bit value (e2 e1 e0)
uint8_t buildByteWithECC(uint8_t baseValue, uint8_t* packetBytes, uint8_t numBytes);

// ECC ERROR DETECTION AND CORRECTION

bool checkByteParity(uint8_t byte); //Check individual byte parity
/**
 * Locate which byte has parity error
 * Returns byte index (0-3) or -1 if no error
 */
int8_t findCorruptedByte(uint8_t* packet, uint8_t numBytes);
/**
 * Locate corrupted bit within a byte using p1, p2, p3
 * Returns bit index (0-7) or -1 if can't determine
 */
int8_t locateCorruptedBit(uint8_t* intermediate, uint8_t numBytes, 
                          uint8_t corruptedByteIdx, uint8_t receivedECC);
/**
 * Main error correction function for receiver
 * Returns true if packet is valid (with or without correction)
 * Returns false if uncorrectable error detected
 */
bool correctPacketErrors(uint8_t* packet, uint8_t numBytes);

// RESPONSE BUILDING FUNCTIONS (WITH ECC)
void sendDigitalReadResponse(uint8_t pin, uint16_t value); //Build and send digital read response with ECC
void sendAnalogReadResponse(uint8_t pin, uint16_t value); //Build and send analog read response with ECC
void sendRetransmitRequest(); //Send retransmit request with ECC

// DEBUGGING HELPERS
void printBinary(uint8_t byte);
void printPacket(uint8_t* packet, uint8_t numBytes, const char* label);

#endif // STM32_RECEIVER_H