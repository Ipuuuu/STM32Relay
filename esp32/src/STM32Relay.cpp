#include "STM32Relay.h"


namespace Relay{
// Helper: Convert Relay::Packet to raw bytes for transmission
void packetToBytes(const Packet& packet, uint8_t* bytes, int numDataBytes) {
    bytes[0] = *reinterpret_cast<const uint8_t*>(&packet.commandByte);
    bytes[1] = *reinterpret_cast<const uint8_t*>(&packet.port);
    
    if(numDataBytes >= 1) {
        bytes[2] = *reinterpret_cast<const uint8_t*>(&packet.data[0]);
    }
    if(numDataBytes >= 2) {
        bytes[3] = *reinterpret_cast<const uint8_t*>(&packet.data[1]);
    }
}

// Helper: Convert raw bytes to Relay::Packet
void bytesToPacket(const uint8_t* bytes, Packet& packet, int numDataBytes) {
    packet.commandByte = *reinterpret_cast<const CommandByte*>(&bytes[0]);
    packet.port = *reinterpret_cast<const DataByte*>(&bytes[1]);
    
    if(numDataBytes >= 1) {
        packet.data[0] = *reinterpret_cast<const DataByte*>(&bytes[2]);
    }
    if(numDataBytes >= 2) {
        packet.data[1] = *reinterpret_cast<const DataByte*>(&bytes[3]);
    }
}

// Build a 2-byte packet (command + port)
void buildPacket(Packet& packet, CommandByte::COMMAND_TYPE cmd, uint8_t pin) {
    // Build command byte
    packet.commandByte.command = cmd;
    packet.commandByte.parity = parity3(static_cast<uint8_t>(cmd));
    packet.commandByte.sync = 1;
    
    // Build port byte
    packet.port.data = pin & 0b00111111;
    packet.port.parity = parity6(packet.port.data);
    packet.port.sync = 0;
    
    // Calculate ECC for command-port pair
    packet.commandByte.ecc = calculateECC(packet.commandByte, packet.port);
}

// Build a 4-byte packet (command + port + 2 data bytes)
void buildPacket(Packet& packet, CommandByte::COMMAND_TYPE cmd, uint8_t pin, uint16_t value) {
    // Build command byte
    packet.commandByte.command = cmd;
    packet.commandByte.parity = parity3(static_cast<uint8_t>(cmd));
    packet.commandByte.sync = 1;
    
    // Build port byte
    packet.port.data = pin & 0b00111111;
    packet.port.parity = parity6(packet.port.data);
    packet.port.sync = 0;
    
    // Build data[0] - upper 6 bits
    packet.data[0].data = (value >> 6) & 0b00111111;
    packet.data[0].parity = parity6(packet.data[0].data);
    packet.data[0].sync = 0;
    
    // Build data[1] - lower 6 bits
    packet.data[1].data = value & 0b00111111;
    packet.data[1].parity = parity6(packet.data[1].data);
    packet.data[1].sync = 0;
    
    // Calculate ECC for command-port pair
    packet.commandByte.ecc = calculateECC(packet.commandByte, packet.port);
}

// Build a 3-byte packet (command + port + 1 data byte)
// Build a 3-byte packet (command + port + 1 data byte)
void buildPacket(Packet& packet, CommandByte::COMMAND_TYPE cmd, uint8_t pin, uint8_t singleByteValue) {
    // Build command byte
    packet.commandByte.command = cmd;
    packet.commandByte.parity = parity3(static_cast<uint8_t>(cmd));
    packet.commandByte.sync = 1;
    
    // Build port byte
    packet.port.data = pin & 0b00111111;
    packet.port.parity = parity6(packet.port.data);
    packet.port.sync = 0;
    
    // Build data[0] - single byte value 
    packet.data[0].data = singleByteValue & 0b00111111;
    packet.data[0].parity = parity6(packet.data[0].data);
    packet.data[0].sync = 0;
    
    // Calculate ECC for command-port pair
    packet.commandByte.ecc = calculateECC(packet.commandByte, packet.port);
}

STM32Relay::STM32Relay(TDEV *tdev)
    : tdev(tdev){}



STM32Relay& STM32Relay::sendPacket(const Packet& packet, uint8_t addr) {
    int numDataBytes = packet.commandByte.requiresData();
    uint8_t bytes[4];
    
    packetToBytes(packet, bytes, numDataBytes);
    
    // Send all bytes
    tdev->sendByte(bytes[0], addr);
    tdev->sendByte(bytes[1], addr);
    
    if(numDataBytes >= 1) {
        tdev->sendByte(bytes[2], addr);
    }
    if(numDataBytes >= 2) {
        tdev->sendByte(bytes[3], addr);
    }
    return (*this);
}

bool STM32Relay::recvPacket(Packet& packet, int expectedBytes, uint8_t addr) {
    uint8_t bytes[4] = {0};
    
    // Receive bytes
    for(int i = 0; i < expectedBytes; i++) {
        bytes[i] = tdev->recvByte(addr);
        if(bytes[i] == 0xFF) {  // Timeout
            return false;
        }
    }
        
    // Convert to packet structure
    int numDataBytes = expectedBytes - 2;  // Subtract command and port bytes
    bytesToPacket(bytes, packet, numDataBytes);
    
    // Verify sync bit on command byte
    if(packet.commandByte.sync != 1) {
        return false;
    }
    
    // Perform error correction
    auto [errorByte, errorBit] = packet.locateAndCorrectError();
    
    // If error was detected and corrected, you might want to log it
    // (errorByte != 0) means an error was found and corrected
    
    return true;
}

STM32Relay&  STM32Relay::pinMode(uint8_t pin, uint8_t mode, uint8_t addr){
    //debug
    Serial.println("\n=== pinMode Debug ===");
    Serial.print("Pin: "); Serial.println(pin);
    Serial.print("Mode: "); 
    Serial.println((mode == INPUT) ? "INPUT" : 
                    (mode == OUTPUT) ? "OUTPUT" : 
                    (mode == INPUT_PULLUP) ? "INPUT_PULLUP" :
                     "INPUT_PULLDOWN");

    // CommandByte::COMMAND_TYPE cmd;
    // if(mode == 0x03) {
    //     cmd = CommandByte::CMD_SET_PIN_MODE;
    // } else if(mode == 0x01) {
    //     cmd = CommandByte::CMD_SET_PIN_MODE;
    // }
    // else if(mode == 0x05) {
    //     cmd = CommandByte::CMD_SET_PIN_MODE;
    // }
    
    // to resolve mode config confliction on STM32 side
    mode = (mode == INPUT) ? 0x00 : 
           (mode == OUTPUT) ? 0x01 : 
           (mode == INPUT_PULLUP) ? 0x03 :
           (mode == INPUT_PULLDOWN) ? 0x09 : 0x00;
    // Build packet (3 bytes)
    Packet packet;
    buildPacket(packet, CommandByte::CMD_SET_PIN_MODE, pin, mode);

    //debug
    Serial.print("Command: 0b"); Serial.println(static_cast<uint8_t>(packet.commandByte.command), BIN);
    Serial.print("ECC: 0b"); Serial.println(packet.commandByte.ecc, BIN);
    Serial.print("Parity: "); Serial.println(packet.commandByte.parity);
    Serial.print("Port Data: 0x"); Serial.println(packet.port.data, HEX);
    Serial.print("Mode Data: 0x"); Serial.println(packet.data[0].data, HEX);

    sendPacket(packet, addr);
    //debug
    Serial.println("Packet sent");
    
    return (*this);
}



STM32Relay&  STM32Relay::digitalWrite(uint8_t pin, uint8_t value, uint8_t addr){
    //debug
    Serial.println("\n=== digitalWrite Debug ===");
    Serial.print("Pin: "); Serial.println(pin);
    Serial.print("Value: "); Serial.println(value ? "HIGH" : "LOW");

    CommandByte::COMMAND_TYPE cmd;
    if(value == 1) {
        cmd = CommandByte::CMD_D_W_HIGH;
    } else {
        cmd = CommandByte::CMD_D_W_LOW;
    }
    
    Packet packet;
    buildPacket(packet, cmd, pin);

    //debug
    Serial.print("Command: 0b"); Serial.println(static_cast<uint8_t>(packet.commandByte.command), BIN);
    Serial.print("ECC: 0b"); Serial.println(packet.commandByte.ecc, BIN);
    Serial.print("Parity: "); Serial.println(packet.commandByte.parity);
    Serial.print("Port Data: 0x"); Serial.println(packet.port.data, HEX);
    
    sendPacket(packet, addr);
    //debug
    Serial.println("Packet sent");

    return (*this);
}

bool STM32Relay::digitalRead(uint8_t pin, uint8_t addr) {
    Serial.println("\n=== digitalRead Debug ===");
    Serial.print("Pin: "); Serial.println(pin);

    Packet requestPacket;
    buildPacket(requestPacket, CommandByte::CMD_D_R, pin);

    //debug
    Serial.print("Request Command: 0b"); Serial.println(static_cast<uint8_t>(requestPacket.commandByte.command), BIN);
    Serial.print("Request ECC: 0b"); Serial.println(requestPacket.commandByte.ecc, BIN);
    
    sendPacket(requestPacket, addr);
    //debug
    Serial.println("Request sent, waiting for reply...");
    
    // Receive response (2 bytes: command + data)
    Packet replyPacket;
    if(!recvPacket(replyPacket, 2, addr)) {
        return false;
    }

    //debug
    Serial.println("Reply received");
    Serial.print("Reply Command: 0b"); Serial.println(static_cast<uint8_t>(replyPacket.commandByte.command), BIN);
    Serial.print("Reply ECC: 0b"); Serial.println(replyPacket.commandByte.ecc, BIN);
    Serial.print("Reply Parity: "); Serial.println(replyPacket.commandByte.parity);
    
    // Extract reply code from command byte
    uint8_t replyCode = static_cast<uint8_t>(replyPacket.commandByte.command);
    
    //debug
    bool result = (replyCode == REPLY_D_HIGH);
    Serial.print("Result: "); Serial.println(result ? "HIGH" : "LOW");

    return (replyCode == REPLY_D_HIGH);
}

STM32Relay&  STM32Relay::analogWrite(uint8_t pin, uint8_t value, uint8_t addr){
    //debug
    Serial.println("\n=== analogWrite Debug ===");
    Serial.print("Pin: "); Serial.println(pin);
    Serial.print("Value (8-bit): "); Serial.println(value);

    // Convert 8-bit to 12-bit
    uint16_t value12bits = (uint16_t)value << 4;

    //debug
    Serial.print("Value (12-bit): "); Serial.println(value12bits);

    Packet packet;
    buildPacket(packet, CommandByte::CMD_A_W, pin, value12bits);

    //debug
    Serial.print("Command: 0b"); Serial.println(static_cast<uint8_t>(packet.commandByte.command), BIN);
    Serial.print("ECC: 0b"); Serial.println(packet.commandByte.ecc, BIN);
    Serial.print("Port Data: 0x"); Serial.println(packet.port.data, HEX);
    Serial.print("Data[0]: 0b"); Serial.println(packet.data[0].data, BIN);
    Serial.print("Data[1]: 0b"); Serial.println(packet.data[1].data, BIN);
    Serial.print("Data[0] Parity: "); Serial.println(packet.data[0].parity);
    Serial.print("Data[1] Parity: "); Serial.println(packet.data[1].parity);

    sendPacket(packet, addr);

    //debug
    Serial.println("Packet sent");


    return (*this);
}

int STM32Relay::analogRead(uint8_t pin, uint8_t addr){
    //debug
    Serial.println("\n=== analogRead Debug ===");
    Serial.print("Pin: "); Serial.println(pin);

    Packet requestPacket;
    buildPacket(requestPacket, CommandByte::CMD_A_R, pin);

    //debug
    Serial.print("Request Command: 0b"); Serial.println(static_cast<uint8_t>(requestPacket.commandByte.command), BIN);
    Serial.print("Request ECC: 0b"); Serial.println(requestPacket.commandByte.ecc, BIN);

    sendPacket(requestPacket, addr);

    //debug
    Serial.println("Request sent, waiting for reply...");
    
    // Receive response (4 bytes: command + port + 2 data bytes)
    Packet replyPacket;
    if(!recvPacket(replyPacket, 4, addr)) {
        return -1;
    }

    //debug
    Serial.println("Reply received");
    Serial.print("Reply Command: 0b"); Serial.println(static_cast<uint8_t>(replyPacket.commandByte.command), BIN);
    Serial.print("Reply ECC: 0b"); Serial.println(replyPacket.commandByte.ecc, BIN);
    Serial.print("Reply Data[0]: 0b"); Serial.println(replyPacket.data[0].data, BIN);
    Serial.print("Reply Data[1]: 0b"); Serial.println(replyPacket.data[1].data, BIN);
    
    // Reconstruct 12-bit value from data bytes
    uint16_t value = ((replyPacket.data[0].data & 0b00111111) << 6) | 
                     (replyPacket.data[1].data & 0b00111111);
    
    //debug
    Serial.print("Reconstructed Value: "); Serial.println(value);
    
    return value;
}

STM32Relay&  STM32Relay::writePPM(uint8_t pin, uint32_t microseconds, uint8_t addr){
    //debug
    Serial.println("\n=== writePPM Debug ===");
    Serial.print("Pin: "); Serial.println(pin);
    Serial.print("Microseconds: "); Serial.println(microseconds);

    // Build packet (4 bytes)
    Packet packet;
    buildPacket(packet, CommandByte::CMD_SET_PPM, pin, (uint16_t)microseconds);
    
    //debug
    Serial.print("Command: 0b"); Serial.println(static_cast<uint8_t>(packet.commandByte.command), BIN);
    Serial.print("ECC: 0b"); Serial.println(packet.commandByte.ecc, BIN);
    Serial.print("Port Data: 0x"); Serial.println(packet.port.data, HEX);
    Serial.print("Data[0]: 0b"); Serial.println(packet.data[0].data, BIN);
    Serial.print("Data[1]: 0b"); Serial.println(packet.data[1].data, BIN);

    sendPacket(packet, addr);

    //debug
    Serial.println("Packet sent");

    return (*this);
}



}