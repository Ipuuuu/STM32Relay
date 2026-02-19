#include <Arduino.h>
#include <HardwareSerial.h>
#include "ReceiverProtocol.h"
#include "tlib.h"
#include <Wire.h>

#include "config.h"

#ifdef TESTMODE_UART
Receiver::UARTDevice uartDevice{115200, PB9, PB10};
Receiver::ProtocolHandler handler{&uartDevice};
#endif

#ifdef TESTMODE_I2C
Receiver::I2CSlave i2cDevice(I2C_ADDRESS);
Receiver::ProtocolHandler handler(&i2cDevice);
#endif

void setup() {
    // Serial Monitor for debug
#ifdef TESTMODE
    Serial.begin(115200);
    
    // Wait for Serial Monitor to connect
    delay(5000);
    Serial.println("Starting Receiver...");
#endif
    
    
    // start the transmission device and protocol handler
    handler.begin();
}

void loop(){
    handler.recover(); // For I2C slave, recover if stuck
    handler.processIncomingBytes();
}