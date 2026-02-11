#include <Arduino.h>
#include <HardwareSerial.h>
#include "ReceiverProtocol.h"
#include "tlib.h"
#include <Wire.h>


#define I2C_ADDRESS 0x42


//Receiver::UARTDevice uartDevice(115200, PB9, PB10, &Serial);


Receiver::I2CSlave i2cDevice(&Wire, I2C_ADDRESS);


// Test UART Receiver
//Receiver::ProtocolHandler handler(&uartDevice);

// Test I2C Slave
Receiver::ProtocolHandler handler(&i2cDevice);

void setup() {
    //UART Setup
    Serial.begin(115200);
    delay(1000);
    
    handler.begin();
}

void loop(){

    // needed to move the reading logic
    // to the ProtocolHandler class and
    // made it generic for any TDEV protocol
    handler.processIncomingBytes();
}