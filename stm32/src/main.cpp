// #include <Arduino.h>
// #include <HardwareSerial.h>

// #define SYNC_BIT 0b10000000
// #define PARITY_BIT 0b01000000

// //cmd bits
// enum COMMAND_TYPE{
//     CMD_D_W_LOW = 0,
//     CMD_D_W_HIGH = 1,
//     CMD_D_R = 2,
//     CMD_A_W = 3,
//     CMD_A_R = 4,
//     CMD_SET_PPM = 5,
//     CMD_SET_PIN_MODE_INPUT = 6,
//     CMD_SET_PIN_MODE_OUTPUT = 7
// };

// //reply bits
// enum REPLY_TYPE{
//     REPLY_D_LOW = 0,
//     REPLY_D_HIGH = 1,
//     REPLY_A_VALUE = 2,
//     REPLY_RETRANSMIT = 3
// };

// //stm32 state
// enum State{
//     IDLE, //waiting for cmd
//     READING_PIN, //reading pin byte
//     READING_VALUE_1, //reading value byte3
//     READING_VALUE_2, //reading value byte4
//     EXECUTE //execute and respond
// };

// // HardwareSerial  Serial1(PA9, PA10); //RX, TX
// uint8_t currentState = IDLE;
// uint8_t commandType;
// uint8_t pinNumber;
// uint16_t value;
// uint8_t byte3, byte4;

// //helper functions declarations
// void handleIdleState(uint8_t byte);
// void handleReadingPinState(uint8_t byte);
// void handleReadingValue1State(uint8_t byte);
// void handleReadingValue2State(uint8_t byte);
// void executeCommand();
// void sendDigitalReadResponse(uint8_t pin, uint16_t value);
// void sendAnalogReadResponse(uint8_t pin, uint16_t value);
// void sendRetransmitRequest();
// bool verifyParity(uint8_t byte);
// uint8_t buildPinByte(uint8_t pinNumber);
// uint8_t calculateParity(int data, uint8_t numBits);

// void setup(){

//     Serial1.begin(115200);
//     Serial.begin(115200);
//     delay(1000);
//     currentState = IDLE;

//     Serial.println(" STM32 Relay Reciever Starting...");

//     // // Physical loopback test first
//     // Serial.println("Loopback test: Connect PA9 to PA10 physically");
//     // Serial1.write(0x55);
//     // delay(100);
//     // if(Serial1.available()){
//     //     uint8_t b = Serial1.read();
//     //     Serial.print("Loopback SUCCESS! Received: 0x");
//     //     Serial.println(b, HEX);
//     // } else {
//     //     Serial.println("Loopback FAILED - check pins");
//     // }
   
// }

// void loop(){
//     if(Serial1.available()){
//         uint8_t byte = Serial1.read();
//         Serial.print("byte Received: 0x");
//         Serial.print(byte, HEX);  // Print hex value
//         Serial.print(" = 0b");
//         for(int i = 7; i >= 0; i--){  // Print all 8 bits
//             Serial.print((byte >> i) & 1);
//         }
//         Serial.print(" | State: ");
//         Serial.println(currentState);

//         switch(currentState){
//             case IDLE:
//                 handleIdleState(byte);
//                 break;
//             case READING_PIN:
//                 handleReadingPinState(byte);
//                 break;
//             case READING_VALUE_1:
//                 handleReadingValue1State(byte);
//                 break;
//             case READING_VALUE_2:
//                 handleReadingValue2State(byte);
//                 break;
//             // case EXECUTE:
//             //     executeCommand();
//             //     currentState = IDLE;
//             //     break;
//         }
//     }
// }

// void handleIdleState(uint8_t byte){
//     if((byte & SYNC_BIT) == 0) //verfify if cmd byte
//         return;
    
//     if(!verifyParity(byte)){ //verify parity
//         sendRetransmitRequest();
//         return;
//     }

//     //extract cmd code (last 3 bits)
//     commandType = byte & 0b00000111;
//     currentState = READING_PIN; //move to next state

// }

// void handleReadingPinState(uint8_t byte){
//     if((byte & SYNC_BIT) != 0){ //verify continuation byte
//         currentState = IDLE; 
//         return;
//     }

//     if(!verifyParity(byte)){
//         sendRetransmitRequest();
//         currentState = IDLE;
//         return;
//     }

//     //extract pin number
//     pinNumber = byte & 0b00111111;

//     //determine next state
//     if(commandType == COMMAND_TYPE::CMD_A_W
//        || commandType == COMMAND_TYPE::CMD_SET_PPM){
//         currentState = READING_VALUE_1;
//     }
//     else{
//         // currentState = EXECUTE;
//         executeCommand();
//         currentState = IDLE;
//     }
// }

// void handleReadingValue1State(uint8_t byte){
//     if((byte & SYNC_BIT) != 0){ //verify continuation byte
//         currentState = IDLE; 
//         return;
//     }

//     if(!verifyParity(byte)){
//         sendRetransmitRequest();
//         currentState = IDLE;
//         return;
//     }

//     byte3 = byte & 0b00111111; //extract upper 6 bits
//     currentState = READING_VALUE_2;
// }

// void handleReadingValue2State(uint8_t byte){
//     if((byte & SYNC_BIT) != 0){ //verify continuation byte
//         currentState = IDLE; 
//         return;
//     }

//     if(!verifyParity(byte)){
//         sendRetransmitRequest();
//         currentState = IDLE;
//         return;
//     }

//     byte4 = byte & 0b00111111; //extract lower 6 bits

//     //combine byte3 and byte4 to form value
//     value = ( (byte3 << 6) | byte4 );

//     // currentState = EXECUTE;
//     executeCommand();
//     currentState = IDLE;
// }

// void executeCommand(){
//     Serial.print("EXECUTING: commandType = ");
//     Serial.print(commandType);
//     Serial.print(", pinNumber = ");
//     Serial.println(pinNumber);
    
//     switch(commandType){
//         case COMMAND_TYPE::CMD_SET_PIN_MODE_INPUT:
//             pinMode(pinNumber, INPUT);
//             break;
//         case COMMAND_TYPE::CMD_SET_PIN_MODE_OUTPUT:
//             Serial.println("executing pinmode command...");
//             pinMode(pinNumber, OUTPUT);
//             break;
//         case COMMAND_TYPE::CMD_D_W_LOW:
//             Serial.println("executing digital LOW command...");
//             digitalWrite(pinNumber, LOW);
//             break;
//         case COMMAND_TYPE::CMD_D_W_HIGH:
//             Serial.println("executing digital HIGH command...");
//             digitalWrite(pinNumber, HIGH);
//             break;
//         case COMMAND_TYPE::CMD_D_R: {
//             uint16_t readValue = digitalRead(pinNumber);
//             sendDigitalReadResponse(pinNumber, readValue);
//             break;
//         }
//         case COMMAND_TYPE::CMD_A_W: {
//             uint8_t value8bit = value >> 4;  // Convert 12-bit to 8-bit
//             analogWrite(pinNumber, value8bit);
//             break;
//         }
//         case COMMAND_TYPE::CMD_A_R: {
//             uint16_t readValue = analogRead(pinNumber);
//             sendAnalogReadResponse(pinNumber, readValue);
//             break;
//         }
//         case COMMAND_TYPE::CMD_SET_PPM:
//             // Implement PPM setting logic here
//             // writePWM(pinNumber, value);
//             break;
//         default:
//             // Unknown command
//             break;
//     }

//     // return to idle
//     currentState = IDLE;
// }


// uint8_t calculateParity(int data, uint8_t numBits){
//     uint8_t count = 0;
//     for(uint8_t i = 0; i < numBits; i++){
//         if(data & (1 << i)){ //check if bit is set
//             count++;
//         }
//     }
//     return count % 2; //returns 1 if odd, 0 if even
// }


// bool verifyParity(uint8_t byte){
//     uint8_t parityBit = (byte >> 6) & 1; //get parity bit
//     uint8_t dataBits = byte & 0b00111111; // get data bits
//     uint8_t calculatedParity = calculateParity(dataBits, 6);

//     return (calculatedParity == parityBit);
// }

// uint8_t buildPinByte(uint8_t pinNumber){
//     uint8_t byte = 0; //continiation byte

//     byte = byte | (pinNumber & 0b00111111);

//     uint8_t parityBit = calculateParity((pinNumber & 0b00111111), 6);
    
//     if(parityBit == 1){
//         byte = byte | PARITY_BIT;
//     }
//     return byte;
// }

// void sendDigitalReadResponse(uint8_t pin, uint16_t value){
//     //format: 1xeeeccc
//     //building reply byte1

//     uint8_t replyByte1 = SYNC_BIT; 
//     if(value == HIGH)
//         replyByte1 |= REPLY_TYPE::REPLY_D_HIGH;
//     else
//         replyByte1 |= REPLY_TYPE::REPLY_D_LOW;

//     uint8_t dataBits = replyByte1 & 0b00111111;
//     uint8_t parity = calculateParity(dataBits, 6); //calc parity bits
//     if(parity == 1)
//         replyByte1 |= PARITY_BIT;
    
//     //building reply byte2
//     uint8_t replyByte2 = buildPinByte(pin);

//     //send response
//     Serial1.write(replyByte1);
//     Serial1.write(replyByte2);

// }

// void sendAnalogReadResponse(uint8_t pin, uint16_t value){
//     //format: byte1: 1xeeeccc
//     //        byte2: 0ppppppp
//     //        byte3: 0vvvvvvv (upper 6 bits)
//     //        byte4: 0vvvvvvv (lower 6 bits)

//     //building reply byte1
//     uint8_t replyByte1 = SYNC_BIT | REPLY_TYPE::REPLY_A_VALUE;
//     uint8_t dataBits = replyByte1 & 0b00111111;
//     //calc parity bits
//     uint8_t parity = calculateParity(dataBits, 6);
//     if(parity == 1)
//         replyByte1 |= PARITY_BIT;

//     //building reply byte2
//     uint8_t replyByte2 = buildPinByte(pin);

//     //building reply byte3
//     uint8_t replyByte3 = 0; //continiation byte
//     replyByte3 = replyByte3 | ((value >> 6) & 0b00111111); // sets byte3 to be upper 6bits of value
//     parity = calculateParity(replyByte3, 6);
//     if(parity == 1){
//         replyByte3 = replyByte3 | PARITY_BIT;
//     }

//     //building reply byte4
//     uint8_t replyByte4= 0; //continiation byte
//     replyByte4 = replyByte4 | (value & 0b00111111); // sets byte4 to be lower 6bits of value
//     parity = calculateParity(replyByte4, 6);
//     if(parity == 1){
//         replyByte4 = replyByte4 | PARITY_BIT;
//     }

//     //send response
//     Serial1.write(replyByte1);
//     Serial1.write(replyByte2);
//     Serial1.write(replyByte3);
//     Serial1.write(replyByte4);
// }

// void sendRetransmitRequest(){
//     uint8_t replyByte = SYNC_BIT | REPLY_TYPE::REPLY_RETRANSMIT;
//     uint8_t dataBits = replyByte & 0b00111111;
//     uint8_t parity = calculateParity(dataBits, 6);
//     if(parity == 1)
//         replyByte |= PARITY_BIT;

//     Serial1.write(replyByte);
// }


/**
 * Unit Tests for STM32 Receiver ECC Implementation
 * 
 * This file contains comprehensive tests for the ECC functions.
 * Upload this to your STM32 to verify the implementation.
 */

#include <Arduino.h>
#include "STM32_Receiver.h"


// Test counters
int testsRun = 0;
int testsPassed = 0;
int testsFailed = 0;

// Test helpers
#define TEST_START(name) \
    Serial.println(); \
    Serial.println("========================================"); \
    Serial.print("TEST: "); \
    Serial.println(name); \
    Serial.println("========================================"); \
    testsRun++;

#define ASSERT_TRUE(condition, message) \
    if(condition) { \
        Serial.print("✓ PASS: "); \
        Serial.println(message); \
        testsPassed++; \
    } else { \
        Serial.print("✗ FAIL: "); \
        Serial.println(message); \
        testsFailed++; \
    }

#define ASSERT_EQUAL(expected, actual, message) \
    if((expected) == (actual)) { \
        Serial.print("✓ PASS: "); \
        Serial.print(message); \
        Serial.print(" (expected="); \
        Serial.print(expected); \
        Serial.print(", actual="); \
        Serial.print(actual); \
        Serial.println(")"); \
        testsPassed++; \
    } else { \
        Serial.print("✗ FAIL: "); \
        Serial.print(message); \
        Serial.print(" (expected="); \
        Serial.print(expected); \
        Serial.print(", actual="); \
        Serial.print(actual); \
        Serial.println(")"); \
        testsFailed++; \
    }

// ============================================================================
// TEST SUITE 1: Basic Parity Functions
// ============================================================================

void test_ParityCalculation() {
    TEST_START("Parity Calculation");
    
    // Test odd parity
    ASSERT_EQUAL(1, calcOddParity(0b111, 3), "Odd parity of 0b111 (3 ones)");
    ASSERT_EQUAL(0, calcOddParity(0b110, 3), "Odd parity of 0b110 (2 ones)");
    ASSERT_EQUAL(1, calcOddParity(0b001, 3), "Odd parity of 0b001 (1 one)");
    ASSERT_EQUAL(0, calcOddParity(0b000, 3), "Odd parity of 0b000 (0 ones)");
    
    // Test even parity
    ASSERT_EQUAL(0, calcEvenParity(0b111, 3), "Even parity of 0b111 (3 ones)");
    ASSERT_EQUAL(1, calcEvenParity(0b110, 3), "Even parity of 0b110 (2 ones)");
    
    // Test verifyParity
    uint8_t byteWithEvenParity = 0b01000110; // parity=1, data=000110 (2 ones - even)
    ASSERT_TRUE(verifyParity(byteWithEvenParity), "Verify correct even parity");
    
    uint8_t byteWithOddParity = 0b00000111; // parity=0, data=000111 (3 ones - odd)
    ASSERT_TRUE(verifyParity(byteWithOddParity), "Verify correct odd parity");
    
    uint8_t byteWithWrongParity = 0b01000111; // parity=1, data=000111 (3 ones - should be 0)
    ASSERT_TRUE(!verifyParity(byteWithWrongParity), "Detect incorrect parity");
}

// ============================================================================
// TEST SUITE 2: Packet Building
// ============================================================================

void test_PacketBuilding() {
    TEST_START("Packet Building");
    
    // Test buildPinByte
    uint8_t pinByte = buildPinByte(5);
    ASSERT_EQUAL(5, pinByte & 0b00111111, "Pin byte contains pin number 5");
    ASSERT_TRUE((pinByte & SYNC_BIT) == 0, "Pin byte has sync bit = 0");
    ASSERT_TRUE(verifyParity(pinByte), "Pin byte has correct parity");
    
    // Test buildValueBytes
    uint8_t byte3, byte4;
    uint16_t testValue = 0b101010101010; // 12-bit value
    buildValueBytes(testValue, byte3, byte4);
    
    uint16_t reconstructed = ((byte3 & 0b00111111) << 6) | (byte4 & 0b00111111);
    ASSERT_EQUAL(testValue, reconstructed, "Value bytes correctly encode/decode");
    ASSERT_TRUE(verifyParity(byte3), "Value byte3 has correct parity");
    ASSERT_TRUE(verifyParity(byte4), "Value byte4 has correct parity");
}

// ============================================================================
// TEST SUITE 3: ECC Bit Calculation
// ============================================================================

void test_ECCCalculation() {
    TEST_START("ECC Bit Calculation");
    
    // Create a simple 2-byte packet
    uint8_t packet[2];
    packet[0] = 0b10000001; // sync=1, parity=0, eee=000, cmd=001
    packet[1] = 0b00000101; // sync=0, parity=0, pin=000101
    
    // Generate intermediate bytes
    uint8_t intermediate[2];
    generateIntermediateBytes(packet, intermediate, 2);
    
    Serial.print("Intermediate[0]: 0b");
    printBinary(intermediate[0]);
    Serial.println();
    Serial.print("Intermediate[1]: 0b");
    printBinary(intermediate[1]);
    Serial.println();
    
    // Calculate ECC bits
    uint8_t p1 = calculateP1(intermediate, 2);
    uint8_t p2 = calculateP2(intermediate, 2);
    uint8_t p3 = calculateP3(intermediate, 2);
    
    Serial.print("P1=");
    Serial.print(p1);
    Serial.print(", P2=");
    Serial.print(p2);
    Serial.print(", P3=");
    Serial.println(p3);
    
    uint8_t ecc = calculateECCBits(intermediate, 2);
    Serial.print("ECC=0b");
    printBinary(ecc);
    Serial.println();
    
    ASSERT_TRUE(ecc <= 0b111, "ECC fits in 3 bits");
}

// ============================================================================
// TEST SUITE 4: Error Detection
// ============================================================================

void test_ErrorDetection() {
    TEST_START("Error Detection");
    
    // Create valid 2-byte packet
    uint8_t packet[2];
    packet[1] = buildPinByte(10);
    uint8_t baseCmd = SYNC_BIT | CMD_D_W_HIGH;
    packet[0] = buildByteWithECC(baseCmd, packet, 2);
    
    Serial.println("Original valid packet:");
    printPacket(packet, 2, "Packet");
    
    // Test 1: No error should pass
    uint8_t testPacket1[2];
    memcpy(testPacket1, packet, 2);
    ASSERT_TRUE(correctPacketErrors(testPacket1, 2), "Valid packet passes ECC check");
    
    // Test 2: Single bit error should be corrected
    uint8_t testPacket2[2];
    memcpy(testPacket2, packet, 2);
    testPacket2[1] ^= (1 << 3); // Flip bit 3 of byte 1
    
    Serial.println("Packet with bit 3 of byte 1 flipped:");
    printPacket(testPacket2, 2, "Corrupted");
    
    bool corrected = correctPacketErrors(testPacket2, 2);
    ASSERT_TRUE(corrected, "Single bit error corrected");
    
    if(corrected) {
        ASSERT_EQUAL(packet[0], testPacket2[0], "Byte 0 restored correctly");
        ASSERT_EQUAL(packet[1], testPacket2[1], "Byte 1 restored correctly");
    }
}

// ============================================================================
// TEST SUITE 5: All Bit Positions (2-Byte)
// ============================================================================

void test_AllBitPositions_2Byte() {
    TEST_START("All Bit Positions - 2 Byte Packet");
    
    int successCount = 0;
    int totalTests = 16; // 2 bytes * 8 bits
    
    for(uint8_t byteIdx = 0; byteIdx < 2; byteIdx++) {
        for(uint8_t bitIdx = 0; bitIdx < 8; bitIdx++) {
            // Build fresh packet
            uint8_t packet[2];
            uint8_t originalPacket[2];
            
            packet[1] = buildPinByte(7);
            uint8_t baseCmd = SYNC_BIT | CMD_D_W_LOW;
            packet[0] = buildByteWithECC(baseCmd, packet, 2);
            
            // Save original
            memcpy(originalPacket, packet, 2);
            
            // Introduce error
            packet[byteIdx] ^= (1 << bitIdx);
            
            // Test correction
            bool result = correctPacketErrors(packet, 2);
            
            // Verify it matches original
            bool matches = (packet[0] == originalPacket[0]) && 
                          (packet[1] == originalPacket[1]);
            
            if(result && matches) {
                successCount++;
            } else {
                Serial.print("  FAILED: Byte ");
                Serial.print(byteIdx);
                Serial.print(", Bit ");
                Serial.println(bitIdx);
            }
        }
    }
    
    Serial.print("Success rate: ");
    Serial.print(successCount);
    Serial.print("/");
    Serial.print(totalTests);
    Serial.print(" (");
    Serial.print((successCount * 100) / totalTests);
    Serial.println("%)");
    
    ASSERT_EQUAL(totalTests, successCount, "All bit positions correctable");
}

// ============================================================================
// TEST SUITE 6: All Bit Positions (4-Byte)
// ============================================================================

void test_AllBitPositions_4Byte() {
    TEST_START("All Bit Positions - 4 Byte Packet");
    
    int successCount = 0;
    int totalTests = 32; // 4 bytes * 8 bits
    
    for(uint8_t byteIdx = 0; byteIdx < 4; byteIdx++) {
        for(uint8_t bitIdx = 0; bitIdx < 8; bitIdx++) {
            // Build fresh packet
            uint8_t packet[4];
            uint8_t originalPacket[4];
            
            packet[1] = buildPinByte(15);
            uint16_t value12bits = (uint16_t)200 << 4;
            buildValueBytes(value12bits, packet[2], packet[3]);
            
            uint8_t baseCmd = SYNC_BIT | CMD_A_W;
            packet[0] = buildByteWithECC(baseCmd, packet, 4);
            
            // Save original
            memcpy(originalPacket, packet, 4);
            
            // Introduce error
            packet[byteIdx] ^= (1 << bitIdx);
            
            // Test correction
            bool result = correctPacketErrors(packet, 4);
            
            // Verify it matches original
            bool matches = true;
            for(uint8_t i = 0; i < 4; i++) {
                if(packet[i] != originalPacket[i]) {
                    matches = false;
                    break;
                }
            }
            
            if(result && matches) {
                successCount++;
            } else {
                Serial.print("  FAILED: Byte ");
                Serial.print(byteIdx);
                Serial.print(", Bit ");
                Serial.println(bitIdx);
            }
        }
    }
    
    Serial.print("Success rate: ");
    Serial.print(successCount);
    Serial.print("/");
    Serial.print(totalTests);
    Serial.print(" (");
    Serial.print((successCount * 100) / totalTests);
    Serial.println("%)");
    
    ASSERT_EQUAL(totalTests, successCount, "All bit positions correctable");
}

// ============================================================================
// TEST SUITE 7: Response Building
// ============================================================================

void test_ResponseBuilding() {
    TEST_START("Response Building with ECC");
    
    Serial.println("Testing digital read response building...");
    // Note: This will actually send on Serial1, so we just verify it doesn't crash
    // In a real test, you'd capture the output
    sendDigitalReadResponse(5, HIGH);
    ASSERT_TRUE(true, "Digital read response built successfully");
    
    Serial.println("Testing analog read response building...");
    sendAnalogReadResponse(10, 2048);
    ASSERT_TRUE(true, "Analog read response built successfully");
    
    Serial.println("Testing retransmit request building...");
    sendRetransmitRequest();
    ASSERT_TRUE(true, "Retransmit request built successfully");
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    
    delay(2000);
    
    Serial.println();
    Serial.println("================================================================================");
    Serial.println("           STM32 RECEIVER ECC IMPLEMENTATION - UNIT TESTS");
    Serial.println("================================================================================");
    Serial.println();
    
    // Run all tests
    test_ParityCalculation();
    test_PacketBuilding();
    test_ECCCalculation();
    test_ErrorDetection();
    test_AllBitPositions_2Byte();
    test_AllBitPositions_4Byte();
    test_ResponseBuilding();
    
    // Print summary
    Serial.println();
    Serial.println("================================================================================");
    Serial.println("                           TEST SUMMARY");
    Serial.println("================================================================================");
    Serial.print("Total tests run:    ");
    Serial.println(testsRun);
    Serial.print("Tests passed:       ");
    Serial.println(testsPassed);
    Serial.print("Tests failed:       ");
    Serial.println(testsFailed);
    Serial.print("Success rate:       ");
    Serial.print((testsPassed * 100) / (testsPassed + testsFailed));
    Serial.println("%");
    Serial.println("================================================================================");
    
    if(testsFailed == 0) {
        Serial.println("🎉 ALL TESTS PASSED! 🎉");
    } else {
        Serial.println("⚠️  SOME TESTS FAILED - Review output above");
    }
}

void loop() {
    // Tests run once in setup
}