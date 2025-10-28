#include <Arduino.h>
#include "STM32Relay.h"
#define LED D10

using namespace Relay;

STM32Relay myRelay(STM32Relay::UART, D8, D9); 

void setup(){
    Serial.begin(115200);
    myRelay.begin(115200);
    delay(1000);
    Serial.println("Deneyap-STM32 Relay Starting...");

    myRelay.pinMode(STM32Relay::PB5, OUTPUT);
    // Serial.println("sent pinmode command...");
}

void loop(){
    myRelay.digitalWrite(STM32Relay::PB5, HIGH);
    Serial.println("sent HIGH...");
    delay(200);
    myRelay.digitalWrite(STM32Relay::PB5, LOW);
    Serial.println("sent LOW...");
    delay(200);
}
