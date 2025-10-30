#include "STM32Relay.h"


//helper functions definition
uint8_t calculateParity(int data, uint8_t numBits){
    uint8_t count = 0;
    for(uint8_t i = 0; i < numBits; i++){
        if(data & (1 << i)){ //check if bit is set
            count++;
        }
    }
    return count % 2; //returns 1 if odd, 0 if even
}

bool verifyParity(uint8_t byte){
    uint8_t parityBit = (byte >> 6) & 1; //get parity bit
    uint8_t dataBits = byte & 0b00111111; // get data bits
    uint8_t calculatedParity = calculateParity(dataBits, 6);

    return (calculatedParity == parityBit);
}

//building cmd bytes
uint8_t buildCommandByte(enum COMMAND_TYPE cmd){
    uint8_t byte = SYNC_BIT; //sets bit 7

    byte = byte | (cmd & 0b00000111); //adds cmd to lower 3 bits
    uint8_t dataBits = byte & 0b00111111;
    uint8_t parityBit = calculateParity(dataBits, 6);

    if(parityBit == 1){ // set parity bit
        byte = byte | PARITY_BIT;
    }

    // static int debugCounter = 0;

    // Serial.println("returning command byte...");
    // Serial.print("Cmd Byte"); Serial.print(debugCounter % 2); Serial.print(": "); Serial.println(byte, BIN);

    // debugCounter++;

    return byte;
}

uint8_t buildPinByte(uint8_t pinNumber){
    uint8_t byte = 0; //continiation byte

    // Serial.print("continuation byte: ");
    // Serial.println(byte, BIN);


    byte = byte | (pinNumber & 0b00111111);

    // Serial.print("pinNum byte: ");
    // Serial.println(byte, BIN);

    uint8_t parityBit = calculateParity((pinNumber & 0b00111111), 6);
    
    if(parityBit == 1){
        byte = byte | PARITY_BIT;
    }

    // Serial.print("added parity byte: ");
    // Serial.println(byte, BIN);
    // Serial.print("added parity byte in HEX: ");
    // Serial.println(byte, HEX);

    // static int debugCounter = 0;

    // Serial.println("returning pin byte...");
    // Serial.print("Pin Byte"); Serial.print(debugCounter % 2); Serial.print(": "); Serial.println(byte, BIN);

    // debugCounter++;

    return byte;
}

void buildValueBytes(int value, uint8_t &byte3, uint8_t &byte4){
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
    // Serial.print("pinMode value: "); Serial.println(value);

    if(value == 0x03){
        cmd = COMMAND_TYPE::CMD_SET_PIN_MODE_OUTPUT;
    }
    else if(value == 0x01){
        cmd = COMMAND_TYPE::CMD_SET_PIN_MODE_INPUT;
    }

    //build bytes
    uint8_t byte1 = buildCommandByte(cmd);
    uint8_t byte2 = buildPinByte(pin);

    // Serial.println("Sending pinmode command bytes:");
    // Serial.print("Byte1: ");Serial.println(byte1, BIN);
    // Serial.print("Byte2: ");Serial.println(byte2, BIN);

    //send
    sendByte(byte1);
    sendByte(byte2);

    return (*this);
}

STM32Relay&  STM32Relay::digitalWrite(uint8_t pin, uint8_t value){
    COMMAND_TYPE cmd;
    if(value == 1){
        cmd = COMMAND_TYPE::CMD_D_W_HIGH;
    }
    else{
        cmd = COMMAND_TYPE::CMD_D_W_LOW;
    }

    //build bytes
    uint8_t byte1 = buildCommandByte(cmd);
    uint8_t byte2 = buildPinByte(pin);

    // Serial.println("Sending digital write command bytes:");
    // Serial.print("Byte1: ");Serial.println(byte1, BIN);
    // Serial.print("Byte2: ");Serial.println(byte2, BIN);

    //send
    sendByte(byte1);
    sendByte(byte2);

    return (*this);
}

bool STM32Relay::digitalRead(uint8_t pin) {
    COMMAND_TYPE cmd = COMMAND_TYPE::CMD_D_R;
    uint8_t byte1 = buildCommandByte(cmd);
    uint8_t byte2 = buildPinByte(pin);

    sendByte(byte1);
    sendByte(byte2);

    //wait for response
    uint8_t replyByte1 = recvByte(1000);
    uint8_t replyByte2 = recvByte(1000);

    //verify sync bit
    if((replyByte1 & SYNC_BIT) == 0){
        return -1; //invalid response
    }

    //verify parity
    if( !(verifyParity(replyByte1)) ){
        return -1; //invalid response
    }

    //extract reply code last 2 bits
    uint8_t replyCode = replyByte2 & 0b00000011;

    return (replyCode == REPLY_D_HIGH);
}

STM32Relay&  STM32Relay::analogWrite(uint8_t pin, uint8_t value){
    uint8_t byte1 = buildCommandByte(CMD_A_W);
    uint8_t byte2 = buildPinByte(pin);

    //convert and scale 8-bit to 12bits
    uint16_t value12bits = (uint16_t)value << 4;
    uint8_t byte3, byte4;

    buildValueBytes(value12bits, byte3, byte4);

    sendByte(byte1);
    sendByte(byte2);
    sendByte(byte3);
    sendByte(byte4);

    return (*this);
}

int STM32Relay::analogRead(uint8_t pin){
    uint8_t byte1 = buildCommandByte(CMD_A_R);
    uint8_t byte2 = buildPinByte(pin);

    sendByte(byte1);
    sendByte(byte2);

    //recv 4-byte response
    uint8_t reply1 = recvByte(1000);
    uint8_t reply2 = recvByte(1000);
    uint8_t reply3 = recvByte(1000);
    uint8_t reply4 = recvByte(1000);

    Serial.print("reply1: ");
    Serial.println(reply1, BIN);
    Serial.print("reply2: ");
    Serial.println(reply2, BIN);
    Serial.print("reply3: ");
    Serial.println(reply3, BIN);
    Serial.print("reply4: ");
    Serial.println(reply4, BIN);

    //verify response
    //verify sync bit-cmd byte
    if((reply1 & SYNC_BIT) == 0){
        return -1; //invalid response
    }
    //verify parity-cmd byte
    if( !(verifyParity(reply1)) ){
        return -1; //invalid response
    }

    //verify sync bit-pin byte
    if((reply2 & SYNC_BIT) != 0){
        return -1; //invalid response
    }

    //verify sync bit for byte3 (should be continuation)
    if((reply3 & SYNC_BIT) != 0){
        return -1;
    }

    //verify sync bit for byte4 (should be continuation)  
    if((reply4 & SYNC_BIT) != 0){
        return -1;
    }
    
    //verify parity-pin byte
    if( !(verifyParity(reply2)) ){
        return -1; //invalid response
    }

    //verify parity byte3
    if( !(verifyParity(reply3)) ){
        return -1; //invalid response
    }
    //verify parity-pin byte3
    if( !(verifyParity(reply4)) ){
        return -1; //invalid response
    }

    //reconstruct 12 bit value
    uint16_t value = ((reply3 & 0b00111111) << 6 ) | (reply4 & 0b00111111);

    // Serial.print("Analog read 12-bit value: ");
    // Serial.println(value, DEC);

    return value;
}

STM32Relay&  STM32Relay::writePPM(uint8_t pin, uint32_t microseconds){
    uint8_t byte1 = buildCommandByte(CMD_SET_PPM);
    uint8_t byte2 = buildPinByte(pin);
    uint8_t byte3, byte4;

    buildValueBytes(microseconds, byte3, byte4);

    sendByte(byte1);
    sendByte(byte2);
    sendByte(byte3);
    sendByte(byte4);

    return (*this);
}



