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

void testServo() {
    Serial.println("Testing servo sweep...");
    
    // Attach servo to pin 6
    myRelay.pinMode(STM32Relay::PB3, OUTPUT);
    
    // Sweep from 0° to 180°
    for(int angle = 0; angle <= 180; angle += 10) {
        uint16_t pulse = map(angle, 0, 180, 1000, 2000);
        Serial.print("Angle: ");
        Serial.print(angle);
        Serial.print("° -> ");
        Serial.print(pulse);
        Serial.println(" us");
        
        myRelay.writePPM(6, pulse);
        delay(100);
    }
    
    // Return to center
    myRelay.writePPM(6, 1500);
    Serial.println("Sweep complete!");
}



void setup(){
    Serial.begin(115200);
    myRelay.begin(115200);
    delay(1000);
    Serial.println("Deneyap-STM32 Relay Starting...");

    myRelay.pinMode(STM32Relay::PB5, OUTPUT);
    // myRelay.pinMode(STM32Relay::PB6, INPUT);
    // myRelay.digitalWrite(STM32Relay::PB6, HIGH);// input pull-up


}

void loop(){
    //digitalwrite
    myRelay.digitalWrite(STM32Relay::PB5, HIGH);
    Serial.println("sent HIGH...");
    delay(200);
    myRelay.digitalWrite(STM32Relay::PB5, LOW);
    Serial.println("sent LOW...");
    delay(200);

    // //analogwrite
    // myRelay.analogWrite(STM32Relay::PB5, brightness);
    // brightness = brightness + fadeAmt;
    // if(brightness == 0 || brightness == 255){
    //     fadeAmt = -fadeAmt; 
    // }
    // delay(30);

    // //analogread
    // sensorValue = myRelay.analogRead(0xC0);
    // Serial.print("Analog Read PA0: ");
    // Serial.println(sensorValue);

    // brightness = map(sensorValue, 0, 1023, 0, 255);

    // myRelay.analogWrite(STM32Relay::PB5, brightness);
    // Serial.print("Analog Write PB5: ");
    // Serial.println(brightness);

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

    // static bool tested = false;
    // if(!tested) {
    //     delay(2000);
    //     testServo();
    //     tested = true;
    // }

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
