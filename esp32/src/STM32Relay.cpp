#include "STM32Relay.h"

namespace Relay{

uint8_t Relay::xiBits = 0;

//helper functions definition
bool Relay::verifyParity(uint8_t byte){
    uint8_t parityBit = (byte >> 6) & 1; //get parity bit
    uint8_t dataBits = byte & 0b00111111; // get data bits
    uint8_t calculatedParity = calcParity(dataBits, 6);

    return (calculatedParity == parityBit);
}

uint8_t Relay::calcParity(int data, uint8_t numBits) {
    uint8_t count = 0;
    for(uint8_t i = 0; i < numBits; i++) {
        if(data & (1 << i)) {
            count++;
        }
    }
    return count % 2; // 1 if odd, 0 if even
}

// ECC Functions
//Calculate odd parity (returns XOR of odd bits startig from LSB)
 
uint8_t Relay::calcOddParity(int data, uint8_t numBits) {
    // XOR together every other bit starting from the LSB (bit positions 0,2,4,...)
    uint8_t parity = 0;
    for (uint8_t i = 0; i < numBits; i += 2) {
        parity ^= (uint8_t)((data >> i) & 1);
    }
    return parity;
}

//Calculate even parity  (returns XOR of even bits startig from LSB)
uint8_t Relay::calcEvenParity(int data, uint8_t numBits) {
    // XOR together every other bit starting from the LSB (bit positions 1,3,5,...)
    uint8_t parity = 0;
    for (uint8_t i = 1; i < numBits; i += 2) {
        parity ^= (uint8_t)((data >> i) & 1);
    }
    return parity;
}

// STEP 2: Generate Intermediate Bytes

/*
    Generate intermediate byte representation for ECC calculation
    Format: [odd][even][x1][x2][x3][c][c][c]
    Bit 7: odd parity of ccc bits
    Bit 6: even parity of ccc bits
    Bits 3-5: x1, x2, x3 (xi parity bits from pin and value bytes)
    Bits 0-2: ccc (the three command bits)
    
    For firstByte: includes x1,x2,x3 bits from xiData
 */
uint8_t Relay::generateIntermediateByte(uint8_t originalByte, bool isFirstByte, uint8_t xiData) {
    uint8_t intermediate = 0;
    
    if(isFirstByte) {
        // For byte1: 1xeeeccc -> extract ccc (bits 0-2)
        uint8_t ccc = originalByte & 0b00000111; // bits 0-2 (the three c bits)
        
        // Bit 7: odd parity of bits 0-2 (ccc)
        intermediate |= (calcOddParity(ccc, 3) << 7);
        
        // Bit 6: even parity of bits 0-2 (ccc) 
        intermediate |= (calcEvenParity(ccc, 3) << 6);
        
        // Bits 5-3: x1, x2, x3 from xiBits (the three xi parity bits from pin and value bytes)
        intermediate |= (xiData << 3) ;
        
        // Bits 2-0: copy ccc (the three command bits)
        intermediate |= ccc ;
    }
    else {
        // For byte2-4: 0xpppppp or 0xvvvvvv -> extract 6 data bits
        uint8_t dataBits = originalByte & 0b00111111;
        
        // Bit 7: odd parity of data bits
        intermediate |= (calcOddParity(dataBits, 6) << 7);
        
        // Bit 6: even parity of data bits
        intermediate |= (calcEvenParity(dataBits, 6) << 6);
        
        // Bits 5-0: copy all 6 data bits
        intermediate |= dataBits;
    }
    
    return intermediate;
}

// Generate all 4 intermediate bytes from original packet
 
void Relay::generateIntermediateBytes(uint8_t* original, uint8_t* intermediate, uint8_t numBytes) {
    for(uint8_t i = 0; i < numBytes; i++) {
        if(i == 0) {
            // First byte: pass xiBits
            intermediate[i] = generateIntermediateByte(original[i], true, xiBits);
        } else {
            // Other bytes: no xiBits
            intermediate[i] = generateIntermediateByte(original[i], false, 0);
        }
    }
}

/*
    Generate all 4 intermediate bytes from original packet with explicit xiData
    This version is used by the receiver for error correction where xiData should be 0
 */
void Relay::generateIntermediateBytes(uint8_t* original, uint8_t* intermediate, uint8_t numBytes, uint8_t xiData) {
    for(uint8_t i = 0; i < numBytes; i++) {
        if(i == 0) {
            // First byte: pass xiData (for receiver, this is 0)
            intermediate[i] = generateIntermediateByte(original[i], true, xiData);
        } else {
            // Other bytes: no xiBits
            intermediate[i] = generateIntermediateByte(original[i], false, 0);
        }
    }
}
// STEP 3: Calculate ECC Bits (p1, p2, p3)

//Calculate p1: parity of bits [7,6,5,4] from all bytes
uint8_t Relay::calculateP1(uint8_t* intermediate, uint8_t numBytes) {
    uint8_t bitCount = 0;
    
    for(uint8_t byteIdx = 0; byteIdx < numBytes; byteIdx++) {
        for(uint8_t bitIdx = 0; bitIdx <= 3; bitIdx++) {
            if(intermediate[byteIdx] & (1 << (7 -bitIdx))) {
                bitCount++;
            }
        }
    }
    
    return bitCount % 2;
}

//Calculate p2: parity of bits [7,6,3,2] from all bytes
uint8_t Relay::calculateP2(uint8_t* intermediate, uint8_t numBytes) {
    uint8_t bitCount = 0;
    uint8_t bitsToCheck[] = {7, 6, 3, 2};
    
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

//Calculate p3: parity of bits [5,3,1] from all bytes
 
uint8_t Relay::calculateP3(uint8_t* intermediate, uint8_t numBytes) {
    uint8_t bitCount = 0;
    uint8_t bitsToCheck[] = {5, 3, 1};
    
    for(uint8_t byteIdx = 0; byteIdx < numBytes; byteIdx++) {
        for(uint8_t i = 0; i < 3; i++) {
            uint8_t bitIdx = bitsToCheck[i];
            if(intermediate[byteIdx] & (1 << bitIdx)) {
                bitCount++;
            }
        }
    }
    
    return bitCount % 2;
}

//Calculate all three ECC bits and return as 3-bit value (e2 e1 e0)
uint8_t Relay::calculateECCBits(uint8_t* intermediate, uint8_t numBytes) {
    uint8_t ecc = 0;
    
    ecc |= (calculateP1(intermediate, numBytes) << 2); // e0 (bit 2)
    ecc |= (calculateP2(intermediate, numBytes) << 1); // e1 (bit 1)
    ecc |= calculateP3(intermediate, numBytes);      // e2 (bit 0)


    
    return ecc;
}

//building cmd bytes WITH error correction bits
uint8_t Relay::buildCommandByte(enum COMMAND_TYPE cmd, uint8_t* packetBytes, uint8_t numBytes) {
    
    // First, build the packet without ECC (eee = 000)
    uint8_t byte1 = SYNC_BIT | (cmd & 0b00000111);
    packetBytes[0] = byte1;
    
    // Generate intermediate bytes (this will use and include xiBits in the first byte)
    uint8_t intermediate[4] = {0};
    generateIntermediateBytes(packetBytes, intermediate, numBytes);
    
    // Calculate ECC bits
    uint8_t ecc = calculateECCBits(intermediate, numBytes);
    
    // Insert ECC bits into positions 3-5 of byte1
    byte1 = SYNC_BIT | ((ecc & 0b111) << 3) | (cmd & 0b00000111);
    
    // Calculate and set parity bit for the complete byte1
    uint8_t dataBits = byte1 & 0b00111111;
    uint8_t parityBit = calcOddParity(dataBits, 6);
    if(parityBit == 1) {
        byte1 |= PARITY_BIT;
    }
    
    // Clear xiBits after use
    xiBits = 0;
    
    return byte1;
}

// STEP 5: Receiver - Error Detection and Correction

// Check individual byte parity (x0, x1, x2, x3)
bool Relay::checkByteParity(uint8_t byte) {
    return verifyParity(byte);
}

/*
    Locate which byte has parity error
    Returns byte index (0-3) or -1 if no error
 */
int8_t Relay::findCorruptedByte(uint8_t* packet, uint8_t numBytes) {
    for(uint8_t i = 0; i < numBytes; i++) {
        if(!checkByteParity(packet[i])) {
            return i;
        }
    }
    return -1; // No parity error found
}

/*
    Locate corrupted bit within a byte using p1, p2, p3
    Returns bit index (0-7) or -1 if can't determine
 */
int8_t Relay::locateCorruptedBit(uint8_t intermediate[], uint8_t numBytes, uint8_t corruptedByteIdx,
                          uint8_t receivedECC) {
    // Recalculate ECC
    uint8_t calculatedECC = calculateECCBits(intermediate, numBytes);
    
    // Extract individual p values
    bool p3Fail = ((receivedECC & 0b001) != (calculatedECC & 0b001));
    bool p2Fail = ((receivedECC & 0b010) != (calculatedECC & 0b010));
    bool p1Fail = ((receivedECC & 0b100) != (calculatedECC & 0b100));
    
    // locate the bit within the corrupted byte
    // We need to look at the CORRUPTED BYTE's intermediate representation
    // uint8_t corruptedIntermediate = intermediate[corruptedByteIdx];

    // Determine bit position using intersection logic
    uint8_t possibleBits = 0xFF; // Start with all (255)bits possible
    
    // p1 check: bits [0,1,2,3] vs [4,5,6,7]
    if(p1Fail) {
        possibleBits &= 0b00001111; // bits 3-0
        
    } else {
        possibleBits &= 0b11110000; // bits 4-7
    }
    
    // p2 check: bits [0,1,4,5] vs [2,3,6,7]
    if(p2Fail) {
        possibleBits &= 0b00110011; // bits 0,1,4,5
    } else {
        possibleBits &= 0b11001100; // bits 2,3,6,7
    }
    
    // p3 check: bits [0,2,4] vs [1,3,5]
    if(p3Fail) {
        // possibleBits &= 0b00101010; // bits 1,3,5
        possibleBits &= 0b10101010; // bits 1,3,5,7
    } else {
        // possibleBits &= 0b00010101; // bits 0,2,4   
        possibleBits &= 0b01010101; // bits 0,2,4,6     
    }
    
    // Find which bit is set in possibleBits
    for(uint8_t bit = 0; bit < 8; bit++) {
        if(possibleBits & (1 << bit)) {
            return bit;
        }
    }
    
    return -1; // Couldn't locate
}

/*
    Main error correction function for receiver
    Returns true if packet is valid (with or without correction)
    Returns false if uncorrectable error detected
 */
bool Relay::correctPacketErrors(uint8_t* packet, uint8_t numBytes) {
    // Extract received ECC bits from byte1
    uint8_t receivedECC = (packet[0] >> 3) & 0b111;
    
    // Recalculate xi bits from the received packet
    uint8_t recalculatedXiBits = 0;
    if(numBytes >= 2) {
        // x1 from byte2 (pin byte)
        uint8_t byte2Data = packet[1] & 0b00111111;
        recalculatedXiBits |= (calcOddParity(byte2Data, 6) << 2);
    }
    if(numBytes >= 3) {
        // x2 from byte3 (value upper byte)
        uint8_t byte3Data = packet[2] & 0b00111111;
        recalculatedXiBits |= (calcOddParity(byte3Data, 6) << 1);
    }
    if(numBytes >= 4) {
        // x3 from byte4 (value lower byte)
        uint8_t byte4Data = packet[3] & 0b00111111;
        recalculatedXiBits |= (calcOddParity(byte4Data, 6) << 0);
    }
    
    // Generate intermediate bytes with recalculated xi bits
    uint8_t intermediate[4] = {0};
    generateIntermediateBytes(packet, intermediate, numBytes, recalculatedXiBits);
    
    // Calculate ECC and compare
    uint8_t calculatedECC = calculateECCBits(intermediate, numBytes);
    
    if(receivedECC == calculatedECC) {
        // No error detected
        return true;
    }
    
    // Error detected - try to locate and correct
    int8_t corruptedByteIdx = findCorruptedByte(packet, numBytes);
    
    if(corruptedByteIdx == -1) {
        // No parity error but ECC mismatch - possible multi-bit error
        return false;
    }
    
    int8_t corruptedBitIdx = locateCorruptedBit(intermediate, numBytes, corruptedByteIdx, receivedECC);

    Serial.print("Corrupted Byte Index: ");
    Serial.println(corruptedByteIdx);
    Serial.print("Corrupted Bit Index: ");
    Serial.println(corruptedBitIdx);
    
    if(corruptedBitIdx == -1) {
        // Couldn't locate bit
        return false;
    }
    
    // Flip the corrupted bit in the original packet
    packet[corruptedByteIdx] ^= (1 << corruptedBitIdx);
    
    // Recalculate xi bits from the CORRECTED packet
    recalculatedXiBits = 0;
    if(numBytes >= 2) {
        uint8_t byte2Data = packet[1] & 0b00111111;
        recalculatedXiBits |= (calcOddParity(byte2Data, 6) << 2);
    }
    if(numBytes >= 3) {
        uint8_t byte3Data = packet[2] & 0b00111111;
        recalculatedXiBits |= (calcOddParity(byte3Data, 6) << 1);
    }
    if(numBytes >= 4) {
        uint8_t byte4Data = packet[3] & 0b00111111;
        recalculatedXiBits |= (calcOddParity(byte4Data, 6) << 0);
    }
    
    // Verify correction
    generateIntermediateBytes(packet, intermediate, numBytes, recalculatedXiBits);
    calculatedECC = calculateECCBits(intermediate, numBytes);
    
    if(receivedECC == calculatedECC) {
        // Successfully corrected
        return true;
    }
    
    // Still incorrect - burst error or uncorrectable
    return false;
}

uint8_t Relay::buildPinByte(uint8_t pinNumber){
    uint8_t byte = 0; //continiation byte

    byte = byte | (pinNumber & 0b00111111);

    uint8_t parityBit = calcOddParity((pinNumber & 0b00111111), 6);

    xiBits = xiBits | (parityBit << 2); //set x1_Bits for later use
    
    if(parityBit == 1){
        byte = byte | PARITY_BIT;
    }

    return byte;
}

void Relay::buildValueBytes(int value, uint8_t &byte3, uint8_t &byte4){
    //byte 3
    byte3 = 0; //continiation byte
    byte3 = byte3 | ((value >> 6) & 0b00111111); // sets byte3 to be upper 6bits of value
    uint8_t parityBit = calcOddParity(byte3, 6);
    xiBits = xiBits | (parityBit << 1);//set x2_Bits for later use

    if(parityBit == 1){
        byte3 = byte3 | PARITY_BIT;
    }

    //byte4
    byte4= 0; //continiation byte
    byte4 = byte4 | (value & 0b00111111); // sets byte4 to be lower 6bits of value
    parityBit = calcOddParity(byte4, 6);
    xiBits = xiBits | (parityBit << 0); //set x3_Bits for later use

    if(parityBit == 1){
        byte4 = byte4 | PARITY_BIT;
    }
}


STM32Relay::STM32Relay(commType type, uint8_t rx, uint8_t tx):
comm_Type(type), txPin(tx), rxPin(rx){
    if(type == UART){
        uart_port = &Serial1;
    }
}


STM32Relay&  STM32Relay::begin(int32_t baud) {
    uart_port->begin(baud, SERIAL_8N1,rxPin,txPin);

    // Clear any startup garbage
    while(uart_port->available()) {
        uart_port->read();
    }
    
    return (*this);
}

STM32Relay&  STM32Relay::sendByte(uint8_t byte) {
    uart_port->write(byte);
    return (*this);
}

uint8_t STM32Relay::recvByte(uint32_t timeout) {
    uint32_t startTime = millis();

    while(!uart_port->available()){
        if((millis() - startTime ) > timeout){
            return -1; // return error
        }
    }
    return uart_port->read(); //return byte
}

STM32Relay&  STM32Relay::pinMode(uint8_t pin, uint8_t value){
    COMMAND_TYPE cmd;
    if(value == 0x03) {
        cmd = COMMAND_TYPE::CMD_SET_PIN_MODE_OUTPUT;
    } else if(value == 0x01) {
        cmd = COMMAND_TYPE::CMD_SET_PIN_MODE_INPUT;
    }
    
    // Build packet (2 bytes)
    uint8_t packet[2];
    packet[1] = buildPinByte(pin);
    
    // Build command byte with ECC
    packet[0] = buildCommandByte(cmd, packet, 2);
    
    // Send
    sendByte(packet[0]);
    sendByte(packet[1]);

    return (*this);
}

STM32Relay&  STM32Relay::digitalWrite(uint8_t pin, uint8_t value){
    COMMAND_TYPE cmd;
    if(value == 1) {
        cmd = COMMAND_TYPE::CMD_D_W_HIGH;
    } else {
        cmd = COMMAND_TYPE::CMD_D_W_LOW;
    }
    
    // Build packet (2 bytes)
    uint8_t packet[2];
    packet[1] = buildPinByte(pin);
    
    // Build command byte with ECC
    packet[0] = buildCommandByte(cmd, packet, 2);
    
    // Send
    sendByte(packet[0]);
    sendByte(packet[1]);

    return (*this);
}

bool STM32Relay::digitalRead(uint8_t pin) {
    COMMAND_TYPE cmd = COMMAND_TYPE::CMD_D_R;
    
    // Build and send request packet
    uint8_t requestPacket[2];
    requestPacket[1] = buildPinByte(pin);
    requestPacket[0] = buildCommandByte(cmd, requestPacket, 2);
    
    sendByte(requestPacket[0]);
    sendByte(requestPacket[1]);
    
    // Wait for response (2 bytes)
    uint8_t replyPacket[2];
    replyPacket[0] = recvByte(1000);
    replyPacket[1] = recvByte(1000);
    
    // Apply error correction
    if(!correctPacketErrors(replyPacket, 2)) {
        Serial.println("Error: Uncorrectable packet error in digitalRead");
        return false;
    }
    
    // Verify sync bit
    if((replyPacket[0] & SYNC_BIT) == 0) {
        return false;
    }
    
    // Extract reply code (last 3 bits)
    uint8_t replyCode = replyPacket[0] & 0b00000111;
    
    return (replyCode == REPLY_D_HIGH);
}

STM32Relay&  STM32Relay::analogWrite(uint8_t pin, uint8_t value){
     // Build packet (4 bytes)
    uint8_t packet[4];
    packet[1] = buildPinByte(pin);
    
    // Convert and scale 8-bit to 12bits
    uint16_t value12bits = (uint16_t)value << 4;
    buildValueBytes(value12bits, packet[2], packet[3]);
    
    // Build command byte with ECC
    packet[0] = buildCommandByte(CMD_A_W, packet, 4);
    
    // Send
    sendByte(packet[0]);
    sendByte(packet[1]);
    sendByte(packet[2]);
    sendByte(packet[3]);

    return (*this);
}

int STM32Relay::analogRead(uint8_t pin){
    // Build and send request packet
    uint8_t requestPacket[2];
    requestPacket[1] = buildPinByte(pin);
    requestPacket[0] = buildCommandByte(CMD_A_R, requestPacket, 2);
    
    sendByte(requestPacket[0]);
    sendByte(requestPacket[1]);
    
    // Recv 4-byte response
    uint8_t replyPacket[4];
    replyPacket[0] = recvByte(1000);
    replyPacket[1] = recvByte(1000);
    replyPacket[2] = recvByte(1000);
    replyPacket[3] = recvByte(1000);
    
    // Apply error correction
    if(!correctPacketErrors(replyPacket, 4)) {
        Serial.println("Error: Uncorrectable packet error in analogRead");
        return -1;
    }
    
    // Verify sync bit
    if((replyPacket[0] & SYNC_BIT) == 0) {
        return -1;
    }
    
    // Reconstruct 12 bit value
    uint16_t value = ((replyPacket[2] & 0b00111111) << 6) | (replyPacket[3] & 0b00111111);
    
    return value;
}

STM32Relay&  STM32Relay::writePPM(uint8_t pin, uint32_t microseconds){
    // Build packet (4 bytes)
    uint8_t packet[4];
    packet[1] = buildPinByte(pin);
    buildValueBytes(microseconds, packet[2], packet[3]);
    
    // Build command byte with ECC
    packet[0] = buildCommandByte(CMD_SET_PPM, packet, 4);
    
    // Send
    sendByte(packet[0]);
    sendByte(packet[1]);
    sendByte(packet[2]);
    sendByte(packet[3]);

    return (*this);
}



}