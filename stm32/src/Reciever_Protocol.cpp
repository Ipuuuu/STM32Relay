#include <memory>
#include "ReceiverProtocol.h"

namespace Receiver {

ProtocolHandler::ProtocolHandler(TDEV* tdev) 
    : tdev(tdev), currentState(IDLE), expectedDataBytes(0), 
      receivedDataBytes(0), servoCount(0) {
    
    // Initialize servo map
    for(int i = 0; i < 10; i++) {
        servoMap[i].pin = 255;
        servoMap[i].attached = false;
    }
}

void ProtocolHandler::resetState() {
    currentState = IDLE;
    expectedDataBytes = 0;
    receivedDataBytes = 0;
    memset(&currentPacket, 0, sizeof(currentPacket));
}

void ProtocolHandler::processIncomingByte(uint8_t byte) {
#ifdef TESTMODE
    Serial.print("\n[RX] Byte: 0x");
    Serial.print(byte, HEX);
    Serial.print(" = 0b");
    for(int i = 7; i >= 0; i--) {
        Serial.print((byte >> i) & 1);
    }
    Serial.print(" | State: ");
    Serial.println(currentState);
#endif
    
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

void ProtocolHandler::processIncomingBytes() {
    while(tdev->available()) {
        uint8_t byte = tdev->recvByte();
        if(byte != 0xFF) processIncomingByte(byte);
    }
}

void ProtocolHandler::handleIdleState(uint8_t byte) {
    // Verify sync bit (must be 1 for command byte)
    if((byte & 0x80) == 0) {
#ifdef TESTMODE
        Serial.println("[ERROR] Missing sync bit in command byte");
#endif
        return;
    }
    
    // Parse command byte
    currentPacket.commandByte = *reinterpret_cast<Relay::CommandByte*>(&byte);
    
    // Verify parity on command byte
    uint8_t calcParity = Relay::parity3(static_cast<uint8_t>(currentPacket.commandByte.command));
    if(calcParity != currentPacket.commandByte.parity) {
#ifdef TESTMODE
        Serial.println("[ERROR] Command parity mismatch!");
        Serial.print("Expected: "); Serial.print(calcParity);
        Serial.print(", Got: "); Serial.println(currentPacket.commandByte.parity);
#endif
        sendRetransmitRequest();
        resetState();
        return;
    }
#ifdef TESTMODE
    Serial.print("[COMMAND] Type: 0b");
    Serial.print(static_cast<uint8_t>(currentPacket.commandByte.command), BIN);
    Serial.print(", ECC: 0b");
    Serial.println(currentPacket.commandByte.ecc, BIN);
#endif
    
    expectedDataBytes = currentPacket.commandByte.requiresData();
    receivedDataBytes = 0;
    currentState = READING_PORT;
}

void ProtocolHandler::handleReadingPortState(uint8_t byte) {
    // Verify sync bit (must be 0 for data byte)
    if((byte & 0x80) != 0) {
#ifdef TESTMODE
        Serial.println("[ERROR] Unexpected sync bit in port byte");
#endif
        resetState();
        return;
    }
    
    // Parse port byte
    currentPacket.port = *reinterpret_cast<Relay::DataByte*>(&byte);
    
    // Verify parity on port byte
    uint8_t calcParity = Relay::parity6(currentPacket.port.data);
    if(calcParity != currentPacket.port.parity) {
#ifdef TESTMODE
        Serial.println("[ERROR] Port parity mismatch!");
#endif
        sendRetransmitRequest();
        resetState();
        return;
    }
#ifdef TESTMODE
    Serial.print("[PORT] Pin: ");
    Serial.println(currentPacket.port.data);
#endif
    
    // Perform ECC check and correction on command-port pair
    auto [errorByte, errorBit] = currentPacket.locateCorruptBit(
        currentPacket.commandByte, 
        currentPacket.port
    );
    
    if(errorByte != 0) {
#ifdef TESTMODE
        Serial.print("[ECC] Error detected in byte ");
        Serial.print(errorByte);
        Serial.print(", bit ");
        Serial.println(errorBit);
#endif
        
        // Correct the error
        if(errorByte == 1) {
            // Error in command byte
            currentPacket.commandByte.command = static_cast<Relay::CommandByte::COMMAND_TYPE>(
                static_cast<uint8_t>(currentPacket.commandByte.command) ^ (1 << errorBit)
            );
            currentPacket.commandByte.parity = Relay::parity3(
                static_cast<uint8_t>(currentPacket.commandByte.command)
            );
#ifdef TESTMODE
            Serial.println("[ECC] Command byte corrected");
#endif
        } else if(errorByte == 2) {
            // Error in port byte
            currentPacket.port.data ^= (1 << errorBit);
            currentPacket.port.parity = Relay::parity6(currentPacket.port.data);
#ifdef TESTMODE
            Serial.println("[ECC] Port byte corrected");
#endif
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
#ifdef TESTMODE
        Serial.println("[ERROR] Unexpected sync bit in data[0] byte");
#endif
        resetState();
        return;
    }
    
    // Parse data byte
    currentPacket.data[0] = *reinterpret_cast<Relay::DataByte*>(&byte);
    
    // Verify parity
    uint8_t calcParity = Relay::parity6(currentPacket.data[0].data);
    if(calcParity != currentPacket.data[0].parity) {
#ifdef TESTMODE
        Serial.println("[ERROR] Data[0] parity mismatch!");
#endif
        sendRetransmitRequest();
        resetState();
        return;
    }
#ifdef TESTMODE
    Serial.print("[DATA0] Value: 0b");
    Serial.println(currentPacket.data[0].data, BIN);
#endif
    
    // Perform ECC check on command-data[0] pair
    auto [errorByte, errorBit] = currentPacket.locateCorruptBit(
        currentPacket.commandByte,
        currentPacket.data[0]
    );
    
    if(errorByte != 0) {
#ifdef TESTMODE
        Serial.print("[ECC] Error in data[0] pair, byte ");
        Serial.print(errorByte);
        Serial.print(", bit ");
        Serial.println(errorBit);
#endif
        
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
#ifdef TESTMODE
        Serial.println("[ERROR] Unexpected sync bit in data[1] byte");
#endif
        resetState();
        return;
    }
    
    // Parse data byte
    currentPacket.data[1] = *reinterpret_cast<Relay::DataByte*>(&byte);
    
    // Verify parity
    uint8_t calcParity = Relay::parity6(currentPacket.data[1].data);
    if(calcParity != currentPacket.data[1].parity) {
#ifdef TESTMODE
        Serial.println("[ERROR] Data[1] parity mismatch!");
#endif
        sendRetransmitRequest();
        resetState();
        return;
    }
#ifdef TESTMODE
    Serial.print("[DATA1] Value: 0b");
    Serial.println(currentPacket.data[1].data, BIN);
#endif
    
    // Perform ECC check on command-data[1] pair
    auto [errorByte, errorBit] = currentPacket.locateCorruptBit(
        currentPacket.commandByte,
        currentPacket.data[1]
    );
    
    if(errorByte != 0) {
#ifdef TESTMODE
        Serial.print("[ECC] Error in data[1] pair, byte ");
        Serial.print(errorByte);
        Serial.print(", bit ");
        Serial.println(errorBit);
#endif
        
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
    uint8_t pin = currentPacket.port.data;
#ifdef TESTMODE
    Serial.println("\n=== EXECUTING COMMAND ===");
    Serial.print("Command: ");
    Serial.println(static_cast<uint8_t>(currentPacket.commandByte.command));
    Serial.print("Relay Pin: ");
    Serial.print(currentPacket.port.data);
    Serial.print(" -> STM32 Pin: ");
    Serial.println(pin);
#endif
    
    switch(currentPacket.commandByte.command) {
        case Relay::CommandByte::CMD_SET_PIN_MODE: {
            uint8_t mode = currentPacket.data[0].data;
#ifdef TESTMODE
            Serial.print("[EXEC] pinMode -> ");
#endif
            if(mode == 0x01){//deneyaps OUTPUT
                pinMode(pin, OUTPUT);
#ifdef TESTMODE
                Serial.print(mode); Serial.println(" OUTPUT");
#endif
            }
            else if(mode == 0x03){//deneyaps INPUT_PULLUP
                pinMode(pin, INPUT_PULLUP);
#ifdef TESTMODE
                Serial.print(mode); Serial.println(" INPUT_PULLUP");
#endif
            }
            else if(mode == 0x09){// deneyaps INPUT_PULLDOWN
                pinMode(pin, INPUT_PULLDOWN);
#ifdef TESTMODE
                Serial.print(mode); Serial.println(" INPUT_PULLDOWN");
#endif
            }
            else{// same INPUT
                pinMode(pin, mode);
#ifdef TESTMODE
                Serial.print(mode); Serial.println(" INPUT");
#endif
            }
            
            break;
        }
        
        case Relay::CommandByte::CMD_D_W_LOW:
            digitalWrite(pin, LOW);
#ifdef TESTMODE
            Serial.println("[EXEC] digitalWrite -> LOW");
#endif
            break;
            
        case Relay::CommandByte::CMD_D_W_HIGH:
            digitalWrite(pin, HIGH);
#ifdef TESTMODE
            Serial.println("[EXEC] digitalWrite -> HIGH");
#endif
            break;
            
        case Relay::CommandByte::CMD_D_R: {
            bool value = digitalRead(pin);
#ifdef TESTMODE
            Serial.print("[EXEC] digitalRead -> ");
            Serial.println(value ? "HIGH" : "LOW");
#endif
            sendDigitalReadResponse(currentPacket.port.data, value);
            break;
        }
        
        case Relay::CommandByte::CMD_A_W: {
            // Reconstruct 12-bit value
            uint16_t value = ((currentPacket.data[0].data & 0x3F) << 6) | 
                            (currentPacket.data[1].data & 0x3F);
            uint8_t value8bit = value >> 4;  // Convert to 8-bit
            analogWrite(pin, value8bit);
#ifdef TESTMODE
            Serial.print("[EXEC] analogWrite -> ");
            Serial.println(value);
#endif
            break;
        }
        
        case Relay::CommandByte::CMD_A_R: {
            uint16_t value = analogRead(pin);
#ifdef TESTMODE
            Serial.print("[EXEC] analogRead -> ");
            Serial.println(value);
#endif
            sendAnalogReadResponse(currentPacket.port.data, value);
            break;
        }
        
        case Relay::CommandByte::CMD_SET_PPM: {
    uint16_t microseconds = ((currentPacket.data[0].data & 0x3F) << 6) | 
                           (currentPacket.data[1].data & 0x3F);
#ifdef TESTMODE
    Serial.print("[EXEC] writePPM -> Pin ");
    Serial.print(pin);
    Serial.print(", ");
    Serial.print(microseconds);
    Serial.println(" us");
#endif
    
    // Find or attach servo
    int servoIndex = -1;
    for(int i = 0; i < servoCount; i++) {
        if(servoMap[i].pin == pin && servoMap[i].attached) {
            servoIndex = i;
            break;
        }
    }
    
    // Attach if not found
    if(servoIndex == -1) {
        attachServo(pin);
        // Find the newly attached servo
        for(int i = 0; i < servoCount; i++) {
            if(servoMap[i].pin == pin && servoMap[i].attached) {
                servoIndex = i;
                break;
            }
        }
    }
    
    // Write pulse width
    if(servoIndex >= 0) {
        servos[servoIndex].writeMicroseconds(microseconds);
#ifdef TESTMODE
        Serial.println("[SERVO] Position updated");
#endif
    } else {
#ifdef TESTMODE
        Serial.println("[ERROR] Failed to control servo");
#endif
    }
    
    break;
}

        
        default:
#ifdef TESTMODE
            Serial.println("[ERROR] Unknown command!");
#endif
            break;
    }
}

void ProtocolHandler::sendDigitalReadResponse(uint8_t pin, bool value) {
#ifdef TESTMODE
    Serial.println("\n[TX] Sending digitalRead response");
#endif
    
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
    
    tdev->sendByte(byte1);
    tdev->sendByte(byte2);
    
#ifdef TESTMODE
    Serial.print("[TX] Sent: 0x");
    Serial.print(byte1, HEX);
    Serial.print(" 0x");
    Serial.println(byte2, HEX);
#endif
}

void ProtocolHandler::sendAnalogReadResponse(uint8_t pin, uint16_t value) {
#ifdef TESTMODE
    Serial.println("\n[TX] Sending analogRead response");
#endif
    
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
    
    tdev->sendByte(byte1);
    tdev->sendByte(byte2);
    tdev->sendByte(byte3);
    tdev->sendByte(byte4);
    
#ifdef TESTMODE
    Serial.print("[TX] Sent: 0x");
    Serial.print(byte1, HEX);
    Serial.print(" 0x");
    Serial.print(byte2, HEX);
    Serial.print(" 0x");
    Serial.print(byte3, HEX);
    Serial.print(" 0x");
    Serial.println(byte4, HEX);
#endif
}

void ProtocolHandler::sendRetransmitRequest() {
#ifdef TESTMODE
    Serial.println("\n[TX] Sending RETRANSMIT request");
#endif
    
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
    tdev->sendByte(byte);
    
#ifdef TESTMODE
    Serial.print("[TX] Sent: 0x");
    Serial.println(byte, HEX);
#endif
}



void ProtocolHandler::attachServo(uint8_t pin) {
    // Check if already attached
    for(int i = 0; i < servoCount; i++) {
        if(servoMap[i].pin == pin && servoMap[i].attached) {
            return; // Already attached
        }
    }
    
    // Find free slot
    for(int i = 0; i < 10; i++) {
        if(!servoMap[i].attached) {
            servos[i].attach(pin);
            servoMap[i].pin = pin;
            servoMap[i].attached = true;
            if(i >= servoCount) servoCount = i + 1;
#ifdef TESTMODE           
            Serial.print("[SERVO] Attached to pin ");
            Serial.println(pin);
#endif
            return;
        }
    }
#ifdef TESTMODE
    Serial.println("[ERROR] No free servo slots!");
#endif
}

void ProtocolHandler::detachServo(uint8_t pin) {
    for(int i = 0; i < servoCount; i++) {
        if(servoMap[i].pin == pin && servoMap[i].attached) {
            servos[i].detach();
            servoMap[i].attached = false;
#ifdef TESTMODE
            Serial.print("[SERVO] Detached from pin ");
            Serial.println(pin);
#endif
            return;
        }
    }
}


} // namespace Receiver
