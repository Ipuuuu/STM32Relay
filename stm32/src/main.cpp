#include <Arduino.h>
#include <HardwareSerial.h>
#include "ReceiverProtocol.h"
#include "tlib.h"
#include <Wire.h>


#define I2C_ADDRESS 0x42


// Test UART Receiver
Receiver::UARTDevice uartDevice{115200, PB9, PB10};
Receiver::ProtocolHandler handler{&uartDevice};

// Test I2C Slave
//Receiver::I2CSlave i2cDevice(I2C_ADDRESS);
//Receiver::ProtocolHandler handler(&i2cDevice);

void setup() {
    // Serial Monitor for debug
    Serial.begin(115200);
    
    // Wait for Serial Monitor to connect
    delay(100);
    
    // start the transmission device and protocol handler
    handler.begin();
}

void loop(){
    handler.processIncomingBytes();
}