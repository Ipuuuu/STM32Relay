#include <Arduino.h>
#include <HardwareSerial.h>
#include "ReceiverProtocol.h"
#include "commapi.h"
#include <Wire.h>

#include "config.h"

#ifdef TESTMODE_UART
commapi::UARTDevice uartDevice{Serial1, 115200, PB9, PB10};
Receiver::ProtocolHandler handler{&uartDevice};
#endif

#ifdef TESTMODE_I2C
commapi::I2CSlave i2cDevice(Wire, I2C_ADDRESS);
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

    // NEW: Set initial config version (0 means unconfigured)
    handler.setConfigVersion(0);
    // Reset flag is automatically set in constructor
}

void loop(){
    handler.recover(); // For I2C slave, recover if stuck
    handler.processIncomingBytes();

    // Optional: Could clear reset flag after some time if no heartbeat received
    // But better to let master clear it after reconfiguration
}