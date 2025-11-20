#include <Arduino.h>
#include "BoardConfig.h"
#include "STM32Relay.h"
#define LED D10

using namespace Relay;

STM32Relay myRelay(STM32Relay::UART, D8, D9); 
uint8_t brightness= 0;
uint8_t fadeAmt = 5;

int sensorValue = 0;
uint8_t btnState = 0;


void setup(){
    Serial.begin(115200);
    myRelay.begin(115200);
    delay(1000);
    Serial.println("Deneyap-STM32 Relay Starting...");

    myRelay.pinMode(STM32Relay::PB5, OUTPUT);
    myRelay.pinMode(STM32Relay::PB6, INPUT);
    myRelay.digitalWrite(STM32Relay::PB6, HIGH);// input pull-up


}

void loop(){
    //digitalwrite
    // myRelay.analogWrite(STM32Relay::PB5, brightness);
    // Serial.println("sent HIGH...");
    // delay(200);
    // myRelay.digitalWrite(STM32Relay::PB5, LOW);
    // Serial.println("sent LOW...");
    // delay(200);

    //analogwrite
    // myRelay.analogWrite(STM32Relay::PB5, brightness);
    // brightness = brightness + fadeAmt;
    // if(brightness == 0 || brightness == 255){5y
    //     fadeAmt = -fadeAmt; 
    // }
    // delay(30);

    //analogread
    sensorValue = myRelay.analogRead(STM32Relay::PA12);
    Serial.print("Analog Read PA0: ");
    Serial.println(sensorValue);

    brightness = map(sensorValue, 0, 1023, 0, 255);

    myRelay.analogWrite(STM32Relay::PB5, brightness);
    Serial.print("Analog Write PB5: ");
    Serial.println(brightness);

    // //digitalread
    // btnState = myRelay.digitalRead(STM32Relay::PB6);
    // Serial.print("Digital Read PB6: ");
    // Serial.println(btnState);  
    // if(btnState == LOW){
    //     myRelay.digitalWrite(STM32Relay::PB5, HIGH);    }
    // else{
    //     myRelay.digitalWrite(STM32Relay::PB5, LOW);
    // }   

    // //wwritePPM
    // myRelay.writePPM(STM32Relay::PB3, 1500);
    // delay(1000);
    // myRelay.writePPM(STM32Relay::PB3, 2000);
    // delay(1000);
    // myRelay.writePPM(STM32Relay::PB3, 1000);
    // delay(1000);   
    // myRelay.writePPM(STM32Relay::PB3, 500);
    // delay(1000); 
    // myRelay.writePPM(STM32Relay::PB3, 0);
    // delay(1000);  

  //   for (int pos = 500; pos <= 2500; pos += 5) { 
  //   // in steps of 1 degree
  //   myRelay.writePPM(STM32Relay::PB3, pos);              
  //   delay(15);                   
  // }
  // for (int pos = 2500; pos >= 500; pos -= 5) { 
  //   myRelay.writePPM(STM32Relay::PB3, pos);              
  //   delay(15);                       
  // }

}
