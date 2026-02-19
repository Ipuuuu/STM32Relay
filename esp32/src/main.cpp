#include <Arduino.h>
#include "BoardConfig.h"
#include "STM32Relay.h"
#define LED D10

#include "config.h"

using namespace Relay;

#ifdef TESTMODE_UART
UARTDevice uartDev{115200, D8, D9}; // UARTDevice object for Serial1 communication
STM32Relay myRelay{&uartDev};
#endif

#ifdef TESTMODE_I2C
I2CMaster i2cDev{}; // I2CDevice object for I2C communication
STM32Relay myRelay{&i2cDev}; 
#endif





uint8_t brightness= 0;
uint8_t fadeAmt = 5;

int sensorValue = 0;
uint8_t btnState = 0;

void testServo() {
#ifdef TESTMODE
    Serial.println("Testing servo sweep...");
#endif
    
    // Attach servo to pin 6
    myRelay.pinMode(STM32Relay::PB3, OUTPUT);
    
    // Sweep from 0° to 180°
    for(int angle = 0; angle <= 180; angle += 10) {
        uint16_t pulse = map(angle, 0, 180, 1000, 2000);
#ifdef TESTMODE
        Serial.print("Angle: ");
        Serial.print(angle);
        Serial.print("° -> ");
        Serial.print(pulse);
        Serial.println(" us");
#endif
        
        myRelay.writePPM(6, pulse);
        delay(100);
    }
    
    // Return to center
    myRelay.writePPM(6, 1500);
#ifdef TESTMODE
    Serial.println("Sweep complete!");
#endif
}



void setup(){
#ifdef TESTMODE
    Serial.begin(115200);
    delay(3000);
    Serial.println("Deneyap-STM32 Relay Starting...");
#endif

    myRelay.begin();
    delay(100);
    myRelay.pinMode(STM32Relay::PB5, OUTPUT, 0x42);
    //myRelay.pinMode(STM32Relay::PB6, INPUT_PULLUP);


}

void loop(){

#ifdef TESTMODE_DIGITAL_WRITE
        myRelay.digitalWrite(STM32Relay::PB5, HIGH, 0x42);
    #ifdef TESTMODE
        Serial.println("sent HIGH...");
    #endif
        delay(1000);
        myRelay.digitalWrite(STM32Relay::PB5, LOW, 0x42);
    #ifdef TESTMODE
        Serial.println("sent LOW...");
    #endif
        delay(1000);
#endif


#ifdef TESTMODE_ANALOG_WRITE
    myRelay.analogWrite(STM32Relay::PA8, brightness, 0x42);
    brightness = brightness + fadeAmt;
    if(brightness == 0 || brightness == 255){
        fadeAmt = -fadeAmt; 
    }
    delay(30);
#endif

#ifdef TESTMODE_ANALOG_READ
        sensorValue = myRelay.analogRead(0xC0);
    #ifdef TESTMODE
        Serial.print("Analog Read PA0: ");
        Serial.println(sensorValue);
    #endif

        brightness = map(sensorValue, 0, 1023, 0, 255);

        myRelay.analogWrite(STM32Relay::PB5, brightness, 0x42);
    #ifdef TESTMODE
        Serial.print("Analog Write PB5: ");
        Serial.println(brightness);
    #endif
#endif

#ifdef TESTMODE_DIGITAL_READ
        btnState = myRelay.digitalRead(STM32Relay::PB6);
    #ifdef TESTMODE
        Serial.print("Digital Read PB6: ");
        Serial.println(btnState);  
    #endif
        if(btnState == LOW){
            myRelay.digitalWrite(STM32Relay::PB5, HIGH);    
        }
        else{
            myRelay.digitalWrite(STM32Relay::PB5, LOW);
        }
#endif



#ifdef TESTMODE_WRITEPPM
        myRelay.writePPM(STM32Relay::PB3, 1500);
        delay(1000);//digitalread
        btnState = myRelay.digitalRead(STM32Relay::PB6);
    #ifdef TESTMODE
        Serial.print("Digital Read PB6: ");
        Serial.println(btnState);
    #endif
        if(btnState == LOW){
            myRelay.digitalWrite(STM32Relay::PB5, HIGH);
        }
        else{
            myRelay.digitalWrite(STM32Relay::PB5, LOW);
        }   
        myRelay.writePPM(STM32Relay::PB3, 2000);
        delay(1000);
        myRelay.writePPM(STM32Relay::PB3, 1000);
        delay(1000);   
        myRelay.writePPM(STM32Relay::PB3, 500);
        delay(1000); 
        myRelay.writePPM(STM32Relay::PB3, 0);
        delay(1000);  

        static bool tested = false;
        if(!tested) {
            delay(2000);
            testServo();
            tested = true;
        }

        for (int pos = 500; pos <= 2500; pos += 5) { 
        // in steps of 1 degree
        myRelay.writePPM(STM32Relay::PB3, pos);              
        delay(15);                   
    }
    for (int pos = 2500; pos >= 500; pos -= 5) { 
        myRelay.writePPM(STM32Relay::PB3, pos);              
        delay(15);                       
    }
#endif

}
