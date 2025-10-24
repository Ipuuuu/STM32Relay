#include <Arduino.h>
#include "STM32Relay.h"
#define LED D10

using namespace Relay;

STM32Relay myRelay(STM32Relay::UART, D8, D9); 

void setup(){
    Serial.begin(115200);
    myRelay.begin(115200);
    Serial.println("Deneyap STM32 Relay Starting...");

    myRelay.pinMode(STM32Relay::PB5, OUTPUT);
}

void loop(){
    myRelay.digitalWrite(STM32Relay::PB5, HIGH);
    delay(500);
    myRelay.digitalWrite(STM32Relay::PB5, LOW);
    delay(500);
}

