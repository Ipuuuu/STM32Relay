#include <Arduino.h>
#include "STM32Relay.h"
#define LED D10

using namespace Relay;

STM32Relay myRelay(STM32Relay::UART, D8, D9); 
uint8_t brightness= 0;
// uint8_t fadeAmt = 5;

int sensorValue = 0;
uint8_t btnState = 0;


void setup(){
    Serial.begin(115200);
    myRelay.begin(115200);
    delay(1000);
    Serial.println("Deneyap-STM32 Relay Starting...");

    myRelay.pinMode(STM32Relay::PB5, OUTPUT);
    myRelay.pinMode(STM32Relay::PB6, INPUT);


}

void loop(){
    // myRelay.analogWrite(STM32Relay::PB5, brightness);
    // Serial.println("sent HIGH...");
    // delay(200);
    // myRelay.digitalWrite(STM32Relay::PB5, LOW);
    // Serial.println("sent LOW...");
    // delay(200);

    // myRelay.analogWrite(STM32Relay::PB5, brightness);
    // brightness = brightness + fadeAmt;
    // if(brightness == 0 || brightness == 255){
    //     fadeAmt = -fadeAmt; 
    // }
    // delay(30);

    // sensorValue = myRelay.analogRead(STM32Relay::PA12);
    // Serial.print("Analog Read PA0: ");
    // Serial.println(sensorValue);

    // brightness = map(sensorValue, 0, 1023, 0, 255);

    // myRelay.analogWrite(STM32Relay::PB5, brightness);
    // Serial.print("Analog Write PB5: ");
    // Serial.println(brightness);

    btnState = myRelay.digitalRead(STM32Relay::PB6);
    Serial.print("Digital Read PB6: ");
    Serial.println(btnState);  
    if(btnState == HIGH){
        myRelay.digitalWrite(STM32Relay::PB5, HIGH);    }
    else{
        myRelay.digitalWrite(STM32Relay::PB5, LOW);
    }   

}
