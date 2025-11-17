#include <Arduino.h>
#include <HardwareSerial.h>

#define SYNC_BIT 0b10000000
#define PARITY_BIT 0b01000000

//cmd bits
enum COMMAND_TYPE{
    CMD_D_W_LOW = 0,
    CMD_D_W_HIGH = 1,
    CMD_D_R = 2,
    CMD_A_W = 3,
    CMD_A_R = 4,
    CMD_SET_PPM = 5,
    CMD_SET_PIN_MODE_INPUT = 6,
    CMD_SET_PIN_MODE_OUTPUT = 7
};

//reply bits
enum REPLY_TYPE{
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

// HardwareSerial  Serial1(PA9, PA10); //RX, TX
uint8_t currentState = IDLE;
uint8_t commandType;
uint8_t pinNumber;
uint16_t value;
uint8_t byte3, byte4;

//helper functions declarations
void handleIdleState(uint8_t byte);
void handleReadingPinState(uint8_t byte);
void handleReadingValue1State(uint8_t byte);
void handleReadingValue2State(uint8_t byte);
void executeCommand();
void sendDigitalReadResponse(uint8_t pin, uint16_t value);
void sendAnalogReadResponse(uint8_t pin, uint16_t value);
void sendRetransmitRequest();
bool verifyParity(uint8_t byte);
uint8_t buildPinByte(uint8_t pinNumber);
uint8_t calculateParity(int data, uint8_t numBits);

void setup(){

    Serial1.begin(115200);
    Serial.begin(115200);
    delay(1000);
    currentState = IDLE;

    Serial.println(" STM32 Relay Reciever Starting...");

    // // Physical loopback test first
    // Serial.println("Loopback test: Connect PA9 to PA10 physically");
    // Serial1.write(0x55);
    // delay(100);
    // if(Serial1.available()){
    //     uint8_t b = Serial1.read();
    //     Serial.print("Loopback SUCCESS! Received: 0x");
    //     Serial.println(b, HEX);
    // } else {
    //     Serial.println("Loopback FAILED - check pins");
    // }
   
}

void loop(){
    if(Serial1.available()){
        uint8_t byte = Serial1.read();
        Serial.print("byte Received: 0x");
        Serial.print(byte, HEX);  // Print hex value
        Serial.print(" = 0b");
        for(int i = 7; i >= 0; i--){  // Print all 8 bits
            Serial.print((byte >> i) & 1);
        }
        Serial.print(" | State: ");
        Serial.println(currentState);

        switch(currentState){
            case IDLE:
                handleIdleState(byte);
                break;
            case READING_PIN:
                handleReadingPinState(byte);
                break;
            case READING_VALUE_1:
                handleReadingValue1State(byte);
                break;
            case READING_VALUE_2:
                handleReadingValue2State(byte);
                break;
            // case EXECUTE:
            //     executeCommand();
            //     currentState = IDLE;
            //     break;
        }
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

uint8_t buildPinByte(uint8_t pinNumber){
    uint8_t byte = 0; //continiation byte

    byte = byte | (pinNumber & 0b00111111);

    uint8_t parityBit = calculateParity((pinNumber & 0b00111111), 6);
    
    if(parityBit == 1){
        byte = byte | PARITY_BIT;
    }
    return byte;
}

void sendDigitalReadResponse(uint8_t pin, uint16_t value){
    //format: 1xeeeccc
    //building reply byte1

    uint8_t replyByte1 = SYNC_BIT; 
    if(value == HIGH)
        replyByte1 |= REPLY_TYPE::REPLY_D_HIGH;
    else
        replyByte1 |= REPLY_TYPE::REPLY_D_LOW;

    uint8_t dataBits = replyByte1 & 0b00111111;
    uint8_t parity = calculateParity(dataBits, 6); //calc parity bits
    if(parity == 1)
        replyByte1 |= PARITY_BIT;
    
    //building reply byte2
    uint8_t replyByte2 = buildPinByte(pin);

    //send response
    Serial1.write(replyByte1);
    Serial1.write(replyByte2);

}

void sendAnalogReadResponse(uint8_t pin, uint16_t value){
    //format: byte1: 1xeeeccc
    //        byte2: 0ppppppp
    //        byte3: 0vvvvvvv (upper 6 bits)
    //        byte4: 0vvvvvvv (lower 6 bits)

    //building reply byte1
    uint8_t replyByte1 = SYNC_BIT | REPLY_TYPE::REPLY_A_VALUE;
    uint8_t dataBits = replyByte1 & 0b00111111;
    //calc parity bits
    uint8_t parity = calculateParity(dataBits, 6);
    if(parity == 1)
        replyByte1 |= PARITY_BIT;

    //building reply byte2
    uint8_t replyByte2 = buildPinByte(pin);

    //building reply byte3
    uint8_t replyByte3 = 0; //continiation byte
    replyByte3 = replyByte3 | ((value >> 6) & 0b00111111); // sets byte3 to be upper 6bits of value
    parity = calculateParity(replyByte3, 6);
    if(parity == 1){
        replyByte3 = replyByte3 | PARITY_BIT;
    }

    //building reply byte4
    uint8_t replyByte4= 0; //continiation byte
    replyByte4 = replyByte4 | (value & 0b00111111); // sets byte4 to be lower 6bits of value
    parity = calculateParity(replyByte4, 6);
    if(parity == 1){
        replyByte4 = replyByte4 | PARITY_BIT;
    }

    //send response
    Serial1.write(replyByte1);
    Serial1.write(replyByte2);
    Serial1.write(replyByte3);
    Serial1.write(replyByte4);
}

void sendRetransmitRequest(){
    uint8_t replyByte = SYNC_BIT | REPLY_TYPE::REPLY_RETRANSMIT;
    uint8_t dataBits = replyByte & 0b00111111;
    uint8_t parity = calculateParity(dataBits, 6);
    if(parity == 1)
        replyByte |= PARITY_BIT;

    Serial1.write(replyByte);
}