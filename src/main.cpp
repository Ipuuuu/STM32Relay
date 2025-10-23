#include <Arduino.h>
#include <STM32Relay.h>
#define LED D10

using namespace Relay;

STM32Relay relay(STM32::UART, STM32Relay:D8, STM32Relay::D9); //Double check pins

void setup(){
    Serial.begin(115200);
    relay.begin(115200);

    relay.pinMode(STM32Relay::PB5, OUTPUT);
}

void loop(){
    relay.digitalWrite(STM32Relay::PB5, HIGH);
    delay(500);
    relay.digitalWrite(STM32Relay::PB5, LOW);
    delay(500);
}