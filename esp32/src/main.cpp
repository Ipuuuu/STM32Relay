// #include <Arduino.h>
// #include "BoardConfig.h"
// #include "STM32Relay.h"
// #define LED D10

// using namespace Relay;

// STM32Relay myRelay(STM32Relay::UART, D8, D9); 
// uint8_t brightness= 0;
// uint8_t fadeAmt = 5;

// int sensorValue = 0;
// uint8_t btnState = 0;


// void setup(){
//     Serial.begin(115200);
//     myRelay.begin(115200);
//     delay(1000);
//     Serial.println("Deneyap-STM32 Relay Starting...");

//     myRelay.pinMode(STM32Relay::PB5, OUTPUT);
//     myRelay.pinMode(STM32Relay::PB6, INPUT);
//     myRelay.digitalWrite(STM32Relay::PB6, HIGH);// input pull-up


// }

// void loop(){
//     //digitalwrite
//     // myRelay.analogWrite(STM32Relay::PB5, brightness);
//     // Serial.println("sent HIGH...");
//     // delay(200);
//     // myRelay.digitalWrite(STM32Relay::PB5, LOW);
//     // Serial.println("sent LOW...");
//     // delay(200);

//     //analogwrite
//     // myRelay.analogWrite(STM32Relay::PB5, brightness);
//     // brightness = brightness + fadeAmt;
//     // if(brightness == 0 || brightness == 255){5y
//     //     fadeAmt = -fadeAmt; 
//     // }
//     // delay(30);

//     //analogread
//     sensorValue = myRelay.analogRead(STM32Relay::PA12);
//     Serial.print("Analog Read PA0: ");
//     Serial.println(sensorValue);

//     brightness = map(sensorValue, 0, 1023, 0, 255);

//     myRelay.analogWrite(STM32Relay::PB5, brightness);
//     Serial.print("Analog Write PB5: ");
//     Serial.println(brightness);

//     // //digitalread
//     // btnState = myRelay.digitalRead(STM32Relay::PB6);
//     // Serial.print("Digital Read PB6: ");
//     // Serial.println(btnState);  
//     // if(btnState == LOW){
//     //     myRelay.digitalWrite(STM32Relay::PB5, HIGH);    }
//     // else{
//     //     myRelay.digitalWrite(STM32Relay::PB5, LOW);
//     // }   

//     // //wwritePPM
//     // myRelay.writePPM(STM32Relay::PB3, 1500);
//     // delay(1000);
//     // myRelay.writePPM(STM32Relay::PB3, 2000);
//     // delay(1000);
//     // myRelay.writePPM(STM32Relay::PB3, 1000);
//     // delay(1000);   
//     // myRelay.writePPM(STM32Relay::PB3, 500);
//     // delay(1000); 
//     // myRelay.writePPM(STM32Relay::PB3, 0);
//     // delay(1000);  

//   //   for (int pos = 500; pos <= 2500; pos += 5) { 
//   //   // in steps of 1 degree
//   //   myRelay.writePPM(STM32Relay::PB3, pos);              
//   //   delay(15);                   
//   // }
//   // for (int pos = 2500; pos >= 500; pos -= 5) { 
//   //   myRelay.writePPM(STM32Relay::PB3, pos);              
//   //   delay(15);                       
//   // }

// }

// ============================================================================
// ECC TESTING AND DEMONSTRATION CODE
// ============================================================================

#include <Arduino.h>
#include "BoardConfig.h"
#include "STM32Relay.h"

using namespace Relay;
STM32Relay myRelay(STM32Relay::UART, D8, D9); 

// ============================================================================
// Test Helper Functions
// ============================================================================

void printBinary(uint8_t byte) {
    for(int i = 7; i >= 0; i--) {
        Serial.print((byte >> i) & 1);
    }
}

void printPacket(uint8_t* packet, uint8_t numBytes, const char* label) {
    Serial.print(label);
    Serial.print(": ");
    for(uint8_t i = 0; i < numBytes; i++) {
        Serial.print("B");
        Serial.print(i);
        Serial.print("=");
        printBinary(packet[i]);
        Serial.print(" (0x");
        Serial.print(packet[i], HEX);
        Serial.print(") ");
    }
    Serial.println();
}

// ============================================================================
// Test Case 1: 2-Byte Packet (digitalWrite command)
// ============================================================================

void test_2BytePacket_NoError() {
    Serial.println("\n========== Test 1: 2-Byte Packet (No Error) ==========");
    
    // Build a digitalWrite HIGH command for pin 5
    uint8_t packet[2];
    packet[1] = buildPinByte(5);
    packet[0] = buildCommandByte(CMD_D_W_HIGH, packet, 2);
    
    printPacket(packet, 2, "Original packet");
    
    // Test error correction (should pass with no corrections)
    bool result = correctPacketErrors(packet, 2);
    
    Serial.print("Error correction result: ");
    Serial.println(result ? "PASS" : "FAIL");
    
    printPacket(packet, 2, "After correction");
}

void test_2BytePacket_SingleBitError() {
    Serial.println("\n========== Test 2: 2-Byte Packet (Single Bit Error) ==========");
    
    // Build packet
    uint8_t packet[2];
    packet[1] = buildPinByte(5);
    packet[0] = buildCommandByte(CMD_D_W_HIGH, packet, 2);
    
    printPacket(packet, 2, "Original packet");
    
    // Introduce error: flip bit 4 of byte 1
    Serial.println("Introducing error: Flipping bit 4 of byte 1");
    packet[1] ^= (1 << 4);
    
    printPacket(packet, 2, "Corrupted packet");
    
    // Test error correction
    bool result = correctPacketErrors(packet, 2);
    
    Serial.print("Error correction result: ");
    Serial.println(result ? "PASS (corrected)" : "FAIL (uncorrectable)");
    
    printPacket(packet, 2, "After correction");
}

// ============================================================================
// Test Case 2: 4-Byte Packet (analogWrite command)
// ============================================================================

void test_4BytePacket_NoError() {
    Serial.println("\n========== Test 3: 4-Byte Packet (No Error) ==========");
    
    // Build analogWrite command for pin 10, value 128
    uint8_t packet[4];
    packet[1] = buildPinByte(10);
    uint16_t value12bits = (uint16_t)128 << 4;
    buildValueBytes(value12bits, packet[2], packet[3]);
    packet[0] = buildCommandByte(CMD_A_W, packet, 4);
    
    printPacket(packet, 4, "Original packet");
    
    // Test error correction
    bool result = correctPacketErrors(packet, 4);
    
    Serial.print("Error correction result: ");
    Serial.println(result ? "PASS" : "FAIL");
    
    printPacket(packet, 4, "After correction");
}

void test_4BytePacket_SingleBitError() {
    Serial.println("\n========== Test 4: 4-Byte Packet (Single Bit Error) ==========");
    
    // Build packet
    uint8_t packet[4];
    packet[1] = buildPinByte(10);
    uint16_t value12bits = (uint16_t)128 << 4;
    buildValueBytes(value12bits, packet[2], packet[3]);
    packet[0] = buildCommandByte(CMD_A_W, packet, 4);
    
    printPacket(packet, 4, "Original packet");
    
    // Introduce error: flip bit 2 of byte 3
    Serial.println("Introducing error: Flipping bit 2 of byte 3");
    packet[3] ^= (1 << 2);
    
    printPacket(packet, 4, "Corrupted packet");
    
    // Test error correction
    bool result = correctPacketErrors(packet, 4);
    
    Serial.print("Error correction result: ");
    Serial.println(result ? "PASS (corrected)" : "FAIL (uncorrectable)");
    
    printPacket(packet, 4, "After correction");
}

// ============================================================================
// Test Case 3: Multiple Different Bit Errors
// ============================================================================

void test_AllBitPositions_2Byte() {
    Serial.println("\n========== Test 5: Test All Bit Positions (2-Byte) ==========");
    
    int successCount = 0;
    int totalTests = 16; // 2 bytes * 8 bits
    
    for(uint8_t byteIdx = 0; byteIdx < 2; byteIdx++) {
        for(uint8_t bitIdx = 0; bitIdx < 8; bitIdx++) {
            // Build fresh packet
            uint8_t packet[2];
            uint8_t originalPacket[2];
            packet[1] = buildPinByte(7);
            packet[0] = buildCommandByte(CMD_D_W_LOW, packet, 2);
            
            // Save original
            originalPacket[0] = packet[0];
            originalPacket[1] = packet[1];
            
            // Introduce error
            packet[byteIdx] ^= (1 << bitIdx);
            
            // Test correction
            bool result = correctPacketErrors(packet, 2);
            
            // Verify it matches original
            bool matches = (packet[0] == originalPacket[0]) && (packet[1] == originalPacket[1]);
            
            if(result && matches) {
                successCount++;
            } else {
                Serial.print("FAILED: Byte ");
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
}

void test_AllBitPositions_4Byte() {
    Serial.println("\n========== Test 6: Test All Bit Positions (4-Byte) ==========");
    
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
            packet[0] = buildCommandByte(CMD_A_W, packet, 4);
            
            // Save original
            for(uint8_t i = 0; i < 4; i++) {
                originalPacket[i] = packet[i];
            }
            
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
                Serial.print("FAILED: Byte ");
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
}

// ============================================================================
// Main Test Runner
// ============================================================================

void runAllECCTests() {
    Serial.println("\n");
    Serial.println("================================================================================");
    Serial.println("                    STM32 RELAY ECC IMPLEMENTATION TESTS");
    Serial.println("================================================================================");
    
    delay(1000);
    
    test_2BytePacket_NoError();
    delay(500);
    
    test_2BytePacket_SingleBitError();
    delay(500);
    
    test_4BytePacket_NoError();
    delay(500);
    
    test_4BytePacket_SingleBitError();
    delay(500);
    
    test_AllBitPositions_2Byte();
    delay(500);
    
    test_AllBitPositions_4Byte();
    delay(500);
    
    Serial.println("\n");
    Serial.println("================================================================================");
    Serial.println("                         ALL TESTS COMPLETED");
    Serial.println("================================================================================");
}

// ============================================================================
// Arduino Setup/Loop Example
// ============================================================================

void setup() {
    Serial.begin(115200);
    while(!Serial) { delay(10); }
    
    Serial.println("Starting ECC Tests...");
    delay(2000);
    
    runAllECCTests();
}

void loop() {
    // Tests run once in setup
}