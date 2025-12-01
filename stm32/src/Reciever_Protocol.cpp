#include "ReceiverProtocol.h"

namespace Receiver {

ProtocolHandler::ProtocolHandler(HardwareSerial* port) 
    : uart_port(port), currentState(IDLE), expectedDataBytes(0), receivedDataBytes(0) {
}

void ProtocolHandler::begin(uint32_t baud) {
    uart_port->begin(baud);
    
    // Clear any startup garbage
    while(uart_port->available()) {
        uart_port->read();
    }
    
    resetState();
    Serial.println("=== STM32 Receiver Starting ===");
    Serial.println("ECC Error Correction: ENABLED");
}

void ProtocolHandler::resetState() {
    currentState = IDLE;
    expectedDataBytes = 0;
    receivedDataBytes = 0;
    memset(&currentPacket, 0, sizeof(currentPacket));
}

void ProtocolHandler::processIncomingByte(uint8_t byte) {
    Serial.print("\n[RX] Byte: 0x");
    Serial.print(byte, HEX);
    Serial.print(" = 0b");
    for(int i = 7; i >= 0; i--) {
        Serial.print((byte >> i) & 1);
    }
    Serial.print(" | State: ");
    Serial.println(currentState);
    
    switch(currentState) {
        case IDLE:
            handleIdleState(byte);
            break;
        case READING_PORT:
            handleReadingPortState(byte);
            break;
        case READING_DATA_1:
            handleReadingData1State(byte);
            break;
        case READING_DATA_2:
            handleReadingData2State(byte);
            break;
    }
}

void ProtocolHandler::handleIdleState(uint8_t byte) {
    // Verify sync bit (must be 1 for command byte)
    if((byte & 0x80) == 0) {
        Serial.println("[ERROR] Missing sync bit in command byte");
        return;
    }
    
    // Parse command byte
    currentPacket.commandByte = *reinterpret_cast<Relay::CommandByte*>(&byte);
    
    // Verify parity on command byte
    uint8_t calcParity = Relay::parity3(static_cast<uint8_t>(currentPacket.commandByte.command));
    if(calcParity != currentPacket.commandByte.parity) {
        Serial.println("[ERROR] Command parity mismatch!");
        Serial.print("Expected: "); Serial.print(calcParity);
        Serial.print(", Got: "); Serial.println(currentPacket.commandByte.parity);
        sendRetransmitRequest();
        resetState();
        return;
    }
    
    Serial.print("[COMMAND] Type: 0b");
    Serial.print(static_cast<uint8_t>(currentPacket.commandByte.command), BIN);
    Serial.print(", ECC: 0b");
    Serial.println(currentPacket.commandByte.ecc, BIN);
    
    expectedDataBytes = currentPacket.commandByte.requiresData();
    receivedDataBytes = 0;
    currentState = READING_PORT;
}

void ProtocolHandler::handleReadingPortState(uint8_t byte) {
    // Verify sync bit (must be 0 for data byte)
    if((byte & 0x80) != 0) {
        Serial.println("[ERROR] Unexpected sync bit in port byte");
        resetState();
        return;
    }
    
    // Parse port byte
    currentPacket.port = *reinterpret_cast<Relay::DataByte*>(&byte);
    
    // Verify parity on port byte
    uint8_t calcParity = Relay::parity6(currentPacket.port.data);
    if(calcParity != currentPacket.port.parity) {
        Serial.println("[ERROR] Port parity mismatch!");
        sendRetransmitRequest();
        resetState();
        return;
    }
    
    Serial.print("[PORT] Pin: ");
    Serial.println(currentPacket.port.data);
    
    // Perform ECC check and correction on command-port pair
    auto [errorByte, errorBit] = currentPacket.locateCorruptBit(
        currentPacket.commandByte, 
        currentPacket.port
    );
    
    if(errorByte != 0) {
        Serial.print("[ECC] Error detected in byte ");
        Serial.print(errorByte);
        Serial.print(", bit ");
        Serial.println(errorBit);
        
        // Correct the error
        if(errorByte == 1) {
            // Error in command byte
            currentPacket.commandByte.command = static_cast<Relay::CommandByte::COMMAND_TYPE>(
                static_cast<uint8_t>(currentPacket.commandByte.command) ^ (1 << errorBit)
            );
            currentPacket.commandByte.parity = Relay::parity3(
                static_cast<uint8_t>(currentPacket.commandByte.command)
            );
            Serial.println("[ECC] Command byte corrected");
        } else if(errorByte == 2) {
            // Error in port byte
            currentPacket.port.data ^= (1 << errorBit);
            currentPacket.port.parity = Relay::parity6(currentPacket.port.data);
            Serial.println("[ECC] Port byte corrected");
        }
    }
    
    // Determine next state
    if(expectedDataBytes == 0) {
        executeCommand();
        resetState();
    } else if(expectedDataBytes >= 1) {
        currentState = READING_DATA_1;
    }
}

void ProtocolHandler::handleReadingData1State(uint8_t byte) {
    // Verify sync bit (must be 0 for data byte)
    if((byte & 0x80) != 0) {
        Serial.println("[ERROR] Unexpected sync bit in data[0] byte");
        resetState();
        return;
    }
    
    // Parse data byte
    currentPacket.data[0] = *reinterpret_cast<Relay::DataByte*>(&byte);
    
    // Verify parity
    uint8_t calcParity = Relay::parity6(currentPacket.data[0].data);
    if(calcParity != currentPacket.data[0].parity) {
        Serial.println("[ERROR] Data[0] parity mismatch!");
        sendRetransmitRequest();
        resetState();
        return;
    }
    
    Serial.print("[DATA0] Value: 0b");
    Serial.println(currentPacket.data[0].data, BIN);
    
    // Perform ECC check on command-data[0] pair
    auto [errorByte, errorBit] = currentPacket.locateCorruptBit(
        currentPacket.commandByte,
        currentPacket.data[0]
    );
    
    if(errorByte != 0) {
        Serial.print("[ECC] Error in data[0] pair, byte ");
        Serial.print(errorByte);
        Serial.print(", bit ");
        Serial.println(errorBit);
        
        if(errorByte == 1) {
            currentPacket.commandByte.command = static_cast<Relay::CommandByte::COMMAND_TYPE>(
                static_cast<uint8_t>(currentPacket.commandByte.command) ^ (1 << errorBit)
            );
            currentPacket.commandByte.parity = Relay::parity3(
                static_cast<uint8_t>(currentPacket.commandByte.command)
            );
        } else if(errorByte == 2) {
            currentPacket.data[0].data ^= (1 << errorBit);
            currentPacket.data[0].parity = Relay::parity6(currentPacket.data[0].data);
        }
    }
    
    receivedDataBytes++;
    
    if(expectedDataBytes == 1) {
        executeCommand();
        resetState();
    } else {
        currentState = READING_DATA_2;
    }
}

void ProtocolHandler::handleReadingData2State(uint8_t byte) {
    // Verify sync bit (must be 0 for data byte)
    if((byte & 0x80) != 0) {
        Serial.println("[ERROR] Unexpected sync bit in data[1] byte");
        resetState();
        return;
    }
    
    // Parse data byte
    currentPacket.data[1] = *reinterpret_cast<Relay::DataByte*>(&byte);
    
    // Verify parity
    uint8_t calcParity = Relay::parity6(currentPacket.data[1].data);
    if(calcParity != currentPacket.data[1].parity) {
        Serial.println("[ERROR] Data[1] parity mismatch!");
        sendRetransmitRequest();
        resetState();
        return;
    }
    
    Serial.print("[DATA1] Value: 0b");
    Serial.println(currentPacket.data[1].data, BIN);
    
    // Perform ECC check on command-data[1] pair
    auto [errorByte, errorBit] = currentPacket.locateCorruptBit(
        currentPacket.commandByte,
        currentPacket.data[1]
    );
    
    if(errorByte != 0) {
        Serial.print("[ECC] Error in data[1] pair, byte ");
        Serial.print(errorByte);
        Serial.print(", bit ");
        Serial.println(errorBit);
        
        if(errorByte == 1) {
            currentPacket.commandByte.command = static_cast<Relay::CommandByte::COMMAND_TYPE>(
                static_cast<uint8_t>(currentPacket.commandByte.command) ^ (1 << errorBit)
            );
            currentPacket.commandByte.parity = Relay::parity3(
                static_cast<uint8_t>(currentPacket.commandByte.command)
            );
        } else if(errorByte == 2) {
            currentPacket.data[1].data ^= (1 << errorBit);
            currentPacket.data[1].parity = Relay::parity6(currentPacket.data[1].data);
        }
    }
    
    receivedDataBytes++;
    executeCommand();
    resetState();
}

void ProtocolHandler::executeCommand() {
    Serial.println("\n=== EXECUTING COMMAND ===");
    Serial.print("Command: ");
    Serial.println(static_cast<uint8_t>(currentPacket.commandByte.command));
    Serial.print("Pin: ");
    Serial.println(currentPacket.port.data);
    
    switch(currentPacket.commandByte.command) {
        case Relay::CommandByte::CMD_SET_PIN_MODE: {
            // Determine mode from data byte if present
            pinMode(currentPacket.port.data, OUTPUT); // Default to output
            Serial.println("[EXEC] pinMode -> OUTPUT");
            break;
        }
        
        case Relay::CommandByte::CMD_D_W_LOW:
            digitalWrite(currentPacket.port.data, LOW);
            Serial.println("[EXEC] digitalWrite -> LOW");
            break;
            
        case Relay::CommandByte::CMD_D_W_HIGH:
            digitalWrite(currentPacket.port.data, HIGH);
            Serial.println("[EXEC] digitalWrite -> HIGH");
            break;
            
        case Relay::CommandByte::CMD_D_R: {
            bool value = digitalRead(currentPacket.port.data);
            Serial.print("[EXEC] digitalRead -> ");
            Serial.println(value ? "HIGH" : "LOW");
            sendDigitalReadResponse(currentPacket.port.data, value);
            break;
        }
        
        case Relay::CommandByte::CMD_A_W: {
            // Reconstruct 12-bit value
            uint16_t value = ((currentPacket.data[0].data & 0x3F) << 6) | 
                            (currentPacket.data[1].data & 0x3F);
            uint8_t value8bit = value >> 4;  // Convert to 8-bit
            analogWrite(currentPacket.port.data, value8bit);
            Serial.print("[EXEC] analogWrite -> ");
            Serial.println(value);
            break;
        }
        
        case Relay::CommandByte::CMD_A_R: {
            uint16_t value = analogRead(currentPacket.port.data);
            Serial.print("[EXEC] analogRead -> ");
            Serial.println(value);
            sendAnalogReadResponse(currentPacket.port.data, value);
            break;
        }
        
        case Relay::CommandByte::CMD_SET_PPM: {
            uint16_t microseconds = ((currentPacket.data[0].data & 0x3F) << 6) | 
                                   (currentPacket.data[1].data & 0x3F);
            Serial.print("[EXEC] writePPM -> ");
            Serial.println(microseconds);
            // Implement PPM logic here
            break;
        }
        
        default:
            Serial.println("[ERROR] Unknown command!");
            break;
    }
}

void ProtocolHandler::sendDigitalReadResponse(uint8_t pin, bool value) {
    Serial.println("\n[TX] Sending digitalRead response");
    
    Relay::Packet responsePacket;
    
    // Build response command byte
    responsePacket.commandByte.command = value ? 
        static_cast<Relay::CommandByte::COMMAND_TYPE>(Receiver::ProtocolHandler::REPLY_TYPE::REPLY_D_HIGH) :
        static_cast<Relay::CommandByte::COMMAND_TYPE>(Receiver::ProtocolHandler::REPLY_TYPE::REPLY_D_LOW);
    responsePacket.commandByte.parity = Relay::parity3(
        static_cast<uint8_t>(responsePacket.commandByte.command)
    );
    responsePacket.commandByte.sync = 1;
    
    // Build port byte
    responsePacket.port.data = pin & 0x3F;
    responsePacket.port.parity = Relay::parity6(responsePacket.port.data);
    responsePacket.port.sync = 0;
    
    // Calculate ECC
    responsePacket.commandByte.ecc = Relay::calculateECC(
        responsePacket.commandByte,
        responsePacket.port
    );
    
    // Send bytes
    uint8_t byte1 = *reinterpret_cast<uint8_t*>(&responsePacket.commandByte);
    uint8_t byte2 = *reinterpret_cast<uint8_t*>(&responsePacket.port);
    
    uart_port->write(byte1);
    uart_port->write(byte2);
    
    Serial.print("[TX] Sent: 0x");
    Serial.print(byte1, HEX);
    Serial.print(" 0x");
    Serial.println(byte2, HEX);
}

void ProtocolHandler::sendAnalogReadResponse(uint8_t pin, uint16_t value) {
    Serial.println("\n[TX] Sending analogRead response");
    
    Relay::Packet responsePacket;
    
    // Build response command byte
    responsePacket.commandByte.command = static_cast<Relay::CommandByte::COMMAND_TYPE>(
        Receiver::ProtocolHandler::REPLY_TYPE::REPLY_A_VALUE
    );
    responsePacket.commandByte.parity = Relay::parity3(
        static_cast<uint8_t>(responsePacket.commandByte.command)
    );
    responsePacket.commandByte.sync = 1;
    
    // Build port byte
    responsePacket.port.data = pin & 0x3F;
    responsePacket.port.parity = Relay::parity6(responsePacket.port.data);
    responsePacket.port.sync = 0;
    
    // Build data bytes for 12-bit value
    responsePacket.data[0].data = (value >> 6) & 0x3F;
    responsePacket.data[0].parity = Relay::parity6(responsePacket.data[0].data);
    responsePacket.data[0].sync = 0;
    
    responsePacket.data[1].data = value & 0x3F;
    responsePacket.data[1].parity = Relay::parity6(responsePacket.data[1].data);
    responsePacket.data[1].sync = 0;
    
    // Calculate ECC
    responsePacket.commandByte.ecc = Relay::calculateECC(
        responsePacket.commandByte,
        responsePacket.port
    );
    
    // Send bytes
    uint8_t byte1 = *reinterpret_cast<uint8_t*>(&responsePacket.commandByte);
    uint8_t byte2 = *reinterpret_cast<uint8_t*>(&responsePacket.port);
    uint8_t byte3 = *reinterpret_cast<uint8_t*>(&responsePacket.data[0]);
    uint8_t byte4 = *reinterpret_cast<uint8_t*>(&responsePacket.data[1]);
    
    uart_port->write(byte1);
    uart_port->write(byte2);
    uart_port->write(byte3);
    uart_port->write(byte4);
    
    Serial.print("[TX] Sent: 0x");
    Serial.print(byte1, HEX);
    Serial.print(" 0x");
    Serial.print(byte2, HEX);
    Serial.print(" 0x");
    Serial.print(byte3, HEX);
    Serial.print(" 0x");
    Serial.println(byte4, HEX);
}

void ProtocolHandler::sendRetransmitRequest() {
    Serial.println("\n[TX] Sending RETRANSMIT request");
    
    Relay::Packet responsePacket;
    
    responsePacket.commandByte.command = static_cast<Relay::CommandByte::COMMAND_TYPE>(
        Receiver::ProtocolHandler::REPLY_TYPE::REPLY_RETRANSMIT
    );
    responsePacket.commandByte.parity = Relay::parity3(
        static_cast<uint8_t>(responsePacket.commandByte.command)
    );
    responsePacket.commandByte.sync = 1;
    responsePacket.commandByte.ecc = 0; // No ECC needed for single-byte response
    
    uint8_t byte = *reinterpret_cast<uint8_t*>(&responsePacket.commandByte);
    uart_port->write(byte);
    
    Serial.print("[TX] Sent: 0x");
    Serial.println(byte, HEX);
}

} // namespace Receiver
