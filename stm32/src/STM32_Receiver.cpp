#include "STM32_Receiver.h"

HardwareSerial Serial1(PA10, PA9); //RX, TX
uint8_t currentState = IDLE;

uint8_t calculateParity(int data, uint8_t numBits){
    uint8_t count = 0;
    for(uint8_t i = 0; i < numBits; i++){
        if(data & (1 << i)){ //check if bit is set
            count++;
        }
    }
    return count % 2; //returns 1 if odd, 0 if even
}
uint8_t calcOddParity(uint8_t data, uint8_t numBits) {
    uint8_t count = 0;
    for(uint8_t i = 0; i < numBits; i++) {
        if(data & (1 << i)) {
            count++;
        }
    }
    return count % 2; // 1 if odd, 0 if even
}


uint8_t calcEvenParity(uint8_t data, uint8_t numBits) {
    return !calcOddParity(data, numBits);
}

bool verifyParity(uint8_t byte){
    uint8_t parityBit = (byte >> 6) & 1; //get parity bit
    uint8_t dataBits = byte & 0b00111111; // get data bits
    uint8_t calculatedParity = calculateParity(dataBits, 6);

    return (calculatedParity == parityBit);
}

uint8_t buildPinByte(uint8_t pinNumber){
    uint8_t byte = 0; //continiation byte

    byte = byte | (pinNumber & 0b00111111);

    uint8_t parityBit = calculateParity((pinNumber & 0b00111111), 6);
    
    if(parityBit == 1){
        byte = byte | PARITY_BIT;
    }
    return byte;
}

void buildValueBytes(uint16_t value, uint8_t &byte3, uint8_t &byte4){
    //byte 3
    byte3 = 0; //continiation byte
    byte3 = byte3 | ((value >> 6) & 0b00111111); // sets byte3 to be upper 6bits of value
    uint8_t parityBit = calculateParity(byte3, 6);

    if(parityBit == 1){
        byte3 = byte3 | PARITY_BIT;
    }

    //byte4
    byte4= 0; //continiation byte
    byte4 = byte4 | (value & 0b00111111); // sets byte4 to be lower 6bits of value
    parityBit = calculateParity(byte4, 6);

    if(parityBit == 1){
        byte4 = byte4 | PARITY_BIT;
    }
}

void handleIdleState(uint8_t byte){
    if((byte & SYNC_BIT) == 0) //verfify if cmd byte
        return;
    
    if(!verifyParity(byte)){ //verify parity
        sendRetransmitRequest();
        return;
    }

    //extract cmd code (last 3 bits)
    commandType = byte & 0b00000111;
    currentState = READING_PIN; //move to next state

}

void handleReadingPinState(uint8_t byte){
    if((byte & SYNC_BIT) != 0){ //verify continuation byte
        currentState = IDLE; 
        return;
    }

    if(!verifyParity(byte)){
        sendRetransmitRequest();
        currentState = IDLE;
        return;
    }

    //extract pin number
    pinNumber = byte & 0b00111111;

    //determine next state
    if(commandType == COMMAND_TYPE::CMD_A_W
       || commandType == COMMAND_TYPE::CMD_SET_PPM){
        currentState = READING_VALUE_1;
    }
    else{
        // currentState = EXECUTE;
        executeCommand();
        currentState = IDLE;
    }
}

void handleReadingValue1State(uint8_t byte){
    if((byte & SYNC_BIT) != 0){ //verify continuation byte
        currentState = IDLE; 
        return;
    }

    if(!verifyParity(byte)){
        sendRetransmitRequest();
        currentState = IDLE;
        return;
    }

    byte3 = byte & 0b00111111; //extract upper 6 bits
    currentState = READING_VALUE_2;
}

void handleReadingValue2State(uint8_t byte){
    if((byte & SYNC_BIT) != 0){ //verify continuation byte
        currentState = IDLE; 
        return;
    }

    if(!verifyParity(byte)){
        sendRetransmitRequest();
        currentState = IDLE;
        return;
    }

    byte4 = byte & 0b00111111; //extract lower 6 bits

    //combine byte3 and byte4 to form value
    value = ( (byte3 << 6) | byte4 );

    // currentState = EXECUTE;
    executeCommand();
    currentState = IDLE;
}

void executeCommand(){
    Serial.print("EXECUTING: commandType = ");
    Serial.print(commandType);
    Serial.print(", pinNumber = ");
    Serial.println(pinNumber);
    
    switch(commandType){
        case COMMAND_TYPE::CMD_SET_PIN_MODE_INPUT:
            pinMode(pinNumber, INPUT);
            break;
        case COMMAND_TYPE::CMD_SET_PIN_MODE_OUTPUT:
            Serial.println("executing pinmode command...");
            pinMode(pinNumber, OUTPUT);
            break;
        case COMMAND_TYPE::CMD_D_W_LOW:
            Serial.println("executing digital LOW command...");
            digitalWrite(pinNumber, LOW);
            break;
        case COMMAND_TYPE::CMD_D_W_HIGH:
            Serial.println("executing digital HIGH command...");
            digitalWrite(pinNumber, HIGH);
            break;
        case COMMAND_TYPE::CMD_D_R: {
            uint16_t readValue = digitalRead(pinNumber);
            sendDigitalReadResponse(pinNumber, readValue);
            break;
        }
        case COMMAND_TYPE::CMD_A_W: {
            uint8_t value8bit = value >> 4;  // Convert 12-bit to 8-bit
            analogWrite(pinNumber, value8bit);
            break;
        }
        case COMMAND_TYPE::CMD_A_R: {
            uint16_t readValue = analogRead(pinNumber);
            sendAnalogReadResponse(pinNumber, readValue);
            break;
        }
        case COMMAND_TYPE::CMD_SET_PPM:
            // Implement PPM setting logic here
            // writePWM(pinNumber, value);
            break;
        default:
            // Unknown command
            break;
    }

    // return to idle
    currentState = IDLE;
}


void sendDigitalReadResponse(uint8_t pin, uint16_t value) {
    // Build packet (2 bytes)
    uint8_t packet[2];
    packet[1] = buildPinByte(pin);
    
    // Build reply byte with ECC
    uint8_t baseReply = SYNC_BIT;
    if(value == HIGH) {
        baseReply |= REPLY_TYPE::REPLY_D_HIGH;
    } else {
        baseReply |= REPLY_TYPE::REPLY_D_LOW;
    }
    
    packet[0] = buildByteWithECC(baseReply, packet, 2);
    
    // Send response
    Serial1.write(packet[0]);
    Serial1.write(packet[1]);
    
    Serial.println("Sent digital read response with ECC");
}

void sendAnalogReadResponse(uint8_t pin, uint16_t value) {
    // Build packet (4 bytes)
    uint8_t packet[4];
    packet[1] = buildPinByte(pin);
    buildValueBytes(value, packet[2], packet[3]);
    
    // Build reply byte with ECC
    uint8_t baseReply = SYNC_BIT | REPLY_TYPE::REPLY_A_VALUE;
    packet[0] = buildByteWithECC(baseReply, packet, 4);
    
    // Send response
    Serial1.write(packet[0]);
    Serial1.write(packet[1]);
    Serial1.write(packet[2]);
    Serial1.write(packet[3]);
    
    Serial.println("Sent analog read response with ECC");
}

void sendRetransmitRequest() {
    // Build single-byte response (treat as 1-byte packet for now)
    uint8_t replyByte = SYNC_BIT | REPLY_TYPE::REPLY_RETRANSMIT;
    
    // For single byte, just calculate parity
    uint8_t dataBits = replyByte & 0b00111111;
    uint8_t parity = calculateParity(dataBits, 6);
    if(parity == 1) {
        replyByte |= PARITY_BIT;
    }
    
    Serial1.write(replyByte);
    
    Serial.println("Sent retransmit request");
}

uint8_t generateIntermediateByte(uint8_t originalByte, bool isFirstByte) {
    uint8_t intermediate = 0;
    
    if(isFirstByte) {
        // For byte1: 1xeeeccc
        // Extract: ccc (bits 0-2), eee (bits 3-5), x (bit 6), sync (bit 7)
        uint8_t ccc = originalByte & 0b00000111;
        uint8_t eee = (originalByte >> 3) & 0b00000111;
        uint8_t x = (originalByte >> 6) & 1;
        
        // Bit 0: odd parity of ccc (3 bits)
        intermediate |= calcOddParity(ccc, 3);
        
        // Bit 1: even parity of ccc (3 bits)
        intermediate |= (calcEvenParity(ccc, 3) << 1);
        
        // Bits 2-7: x, eee, ccc
        // Reconstruct: [bit7][bit6][bit5][bit4][bit3][bit2][bit1][bit0]
        //              [ccc2][ccc1][ccc0][eee2][eee1][eee0][x][x]
        intermediate |= (x << 2);
        intermediate |= (eee << 3);
        intermediate |= (ccc << 6);
    }
    else {
        // For byte2-4: 0xpppppp or 0xvvvvvv
        uint8_t dataBits = originalByte & 0b00111111; // 6 data bits
        uint8_t parityBit = (originalByte >> 6) & 1;
        
        // Bit 0: odd parity of 6 data bits
        intermediate |= calcOddParity(dataBits, 6);
        
        // Bit 1: even parity of 6 data bits
        intermediate |= (calcEvenParity(dataBits, 6) << 1);
        
        // Bits 2-7: parity bit + 6 data bits
        intermediate |= (parityBit << 2);
        intermediate |= (dataBits << 3);
    }
    
    return intermediate;
}

void generateIntermediateBytes(uint8_t* original, uint8_t* intermediate, uint8_t numBytes) {
    for(uint8_t i = 0; i < numBytes; i++) {
        intermediate[i] = generateIntermediateByte(original[i], i == 0);
    }
}

uint8_t calculateP1(uint8_t* intermediate, uint8_t numBytes) {
    uint8_t bitCount = 0;
    
    // P1: parity of bits [0,1,2,3] from all bytes
    for(uint8_t byteIdx = 0; byteIdx < numBytes; byteIdx++) {
        for(uint8_t bitIdx = 0; bitIdx <= 3; bitIdx++) {
            if(intermediate[byteIdx] & (1 << bitIdx)) {
                bitCount++;
            }
        }
    }
    
    return bitCount % 2;
}

uint8_t calculateP2(uint8_t* intermediate, uint8_t numBytes) {
    uint8_t bitCount = 0;
    
    // P2: parity of bits [0,1,4,5] from all bytes
    uint8_t bitsToCheck[] = {0, 1, 4, 5};
    
    for(uint8_t byteIdx = 0; byteIdx < numBytes; byteIdx++) {
        for(uint8_t i = 0; i < 4; i++) {
            uint8_t bitIdx = bitsToCheck[i];
            if(intermediate[byteIdx] & (1 << bitIdx)) {
                bitCount++;
            }
        }
    }
    
    return bitCount % 2;
}

uint8_t calculateP3(uint8_t* intermediate, uint8_t numBytes) {
    uint8_t bitCount = 0;
    
    // P3: parity of bits [0,2,4,6] from all bytes
    uint8_t bitsToCheck[] = {0, 2, 4, 6};
    
    for(uint8_t byteIdx = 0; byteIdx < numBytes; byteIdx++) {
        for(uint8_t i = 0; i < 4; i++) {
            uint8_t bitIdx = bitsToCheck[i];
            if(intermediate[byteIdx] & (1 << bitIdx)) {
                bitCount++;
            }
        }
    }
    
    return bitCount % 2;
}

uint8_t calculateECCBits(uint8_t* intermediate, uint8_t numBytes) {
    uint8_t ecc = 0;
    
    ecc |= calculateP1(intermediate, numBytes);        // e0 (bit 0)
    ecc |= (calculateP2(intermediate, numBytes) << 1); // e1 (bit 1)
    ecc |= (calculateP3(intermediate, numBytes) << 2); // e2 (bit 2)
    
    return ecc;
}

uint8_t buildByteWithECC(uint8_t baseValue, uint8_t* packetBytes, uint8_t numBytes) {
    // baseValue should already have SYNC_BIT and command/reply bits set
    // We need to calculate ECC and insert it
    
    // Temporarily set ECC bits to 000 for initial calculation
    packetBytes[0] = baseValue & 0b11000111; // Clear eee bits (bits 3-5)
    
    // Generate intermediate bytes
    uint8_t intermediate[4] = {0};
    generateIntermediateBytes(packetBytes, intermediate, numBytes);
    
    // Calculate ECC bits
    uint8_t ecc = calculateECCBits(intermediate, numBytes);
    
    // Insert ECC bits into positions 3-5
    uint8_t byte1 = (baseValue & 0b11000111) | ((ecc & 0b111) << 3);
    
    // Calculate and set parity bit for the complete byte1
    uint8_t dataBits = byte1 & 0b00111111;
    uint8_t parityBit = calculateParity(dataBits, 6);
    if(parityBit == 1) {
        byte1 |= PARITY_BIT;
    }
    
    return byte1;
}

// ============================================================================
// ECC ERROR DETECTION AND CORRECTION
// ============================================================================

bool checkByteParity(uint8_t byte) {
    return verifyParity(byte);
}

int8_t findCorruptedByte(uint8_t* packet, uint8_t numBytes) {
    for(uint8_t i = 0; i < numBytes; i++) {
        if(!checkByteParity(packet[i])) {
            return i;
        }
    }
    return -1; // No parity error found
}

int8_t locateCorruptedBit(uint8_t* intermediate, uint8_t numBytes, 
                          uint8_t corruptedByteIdx, uint8_t receivedECC) {
    // Recalculate ECC
    uint8_t calculatedECC = calculateECCBits(intermediate, numBytes);
    
    // Extract individual p values and check if they fail
    bool p1Fail = ((receivedECC & 0b001) != (calculatedECC & 0b001));
    bool p2Fail = ((receivedECC & 0b010) != (calculatedECC & 0b010));
    bool p3Fail = ((receivedECC & 0b100) != (calculatedECC & 0b100));
    
    // Determine bit position using intersection logic
    uint8_t possibleBits = 0xFF; // Start with all bits possible
    
    // p1 check: bits [0,1,2,3] vs [4,5,6,7]
    if(p1Fail) {
        possibleBits &= 0b11110000; // bits 4-7
    } else {
        possibleBits &= 0b00001111; // bits 0-3
    }
    
    // p2 check: bits [0,1,4,5] vs [2,3,6,7]
    if(p2Fail) {
        possibleBits &= 0b00110011; // bits 0,1,4,5
    } else {
        possibleBits &= 0b11001100; // bits 2,3,6,7
    }
    
    // p3 check: bits [0,2,4,6] vs [1,3,5,7]
    if(p3Fail) {
        possibleBits &= 0b01010101; // bits 0,2,4,6
    } else {
        possibleBits &= 0b10101010; // bits 1,3,5,7
    }
    
    // Find which bit is set in possibleBits
    for(uint8_t bit = 0; bit < 8; bit++) {
        if(possibleBits & (1 << bit)) {
            return bit;
        }
    }
    
    return -1; // Couldn't locate
}

bool correctPacketErrors(uint8_t* packet, uint8_t numBytes) {
    // Extract received ECC bits from byte1 (bits 3-5)
    uint8_t receivedECC = (packet[0] >> 3) & 0b111;
    
    // Generate intermediate bytes
    uint8_t intermediate[4] = {0};
    generateIntermediateBytes(packet, intermediate, numBytes);
    
    // Calculate ECC and compare
    uint8_t calculatedECC = calculateECCBits(intermediate, numBytes);
    
    if(receivedECC == calculatedECC) {
        // No error detected
        return true;
    }
    
    Serial.println("ECC mismatch detected - attempting correction...");
    Serial.print("Received ECC: 0b");
    for(int i = 2; i >= 0; i--) {
        Serial.print((receivedECC >> i) & 1);
    }
    Serial.print(" Calculated ECC: 0b");
    for(int i = 2; i >= 0; i--) {
        Serial.print((calculatedECC >> i) & 1);
    }
    Serial.println();
    
    // Error detected - try to locate and correct
    int8_t corruptedByteIdx = findCorruptedByte(packet, numBytes);
    
    if(corruptedByteIdx == -1) {
        // No parity error but ECC mismatch - possible multi-bit error
        Serial.println("No parity error but ECC mismatch - uncorrectable");
        return false;
    }
    
    Serial.print("Corrupted byte index: ");
    Serial.println(corruptedByteIdx);
    
    int8_t corruptedBitIdx = locateCorruptedBit(intermediate, numBytes, 
                                                 corruptedByteIdx, receivedECC);
    
    if(corruptedBitIdx == -1) {
        Serial.println("Couldn't locate corrupted bit");
        return false;
    }
    
    Serial.print("Corrupted bit index: ");
    Serial.println(corruptedBitIdx);
    
    // Flip the corrupted bit
    packet[corruptedByteIdx] ^= (1 << corruptedBitIdx);
    
    Serial.print("Corrected byte ");
    Serial.print(corruptedByteIdx);
    Serial.print(" to: 0b");
    printBinary(packet[corruptedByteIdx]);
    Serial.println();
    
    // Verify correction by recalculating
    generateIntermediateBytes(packet, intermediate, numBytes);
    calculatedECC = calculateECCBits(intermediate, numBytes);
    
    if(receivedECC == calculatedECC) {
        Serial.println("Error successfully corrected!");
        return true;
    }
    
    // Still incorrect - burst error or uncorrectable
    Serial.println("Correction failed - still incorrect");
    return false;
}

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