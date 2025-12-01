#include <Arduino.h>
#include "ReceiverProtocol.h"

Receiver::ProtocolHandler handler(&Serial1);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    handler.begin(115200);
}

void loop() {
    if(Serial1.available()) {
        uint8_t byte = Serial1.read();
        handler.processIncomingByte(byte);
    }
}