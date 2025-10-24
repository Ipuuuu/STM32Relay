#include <Arduino.h>

#define RX_PIN D8
#define TX_PIN D9

void setup(){
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    
    delay(1000);
    Serial.println("ESP32 UART Test Starting...");
    
    // Physical loopback test
    Serial.println("Physical loopback: Connect D9 to D8");
    
    Serial1.write(0x55);
    delay(100);
    
    if(Serial1.available()){
        uint8_t received = Serial1.read();
        Serial.print("SUCCESS! Received: 0x");
        Serial.println(received, HEX);
    } else {
        Serial.println("FAILED - Nothing received");
    }
}

void loop(){
    // Echo test
    if(Serial.available()){
        char c = Serial.read();
        Serial1.write(c);
        Serial.print("Sent to Serial1: ");
        Serial.println(c);
    }
    
    if(Serial1.available()){
        char c = Serial1.read();
        Serial.print("Received from Serial1: ");
        Serial.println(c);
    }
}

// #include <Arduino.h>
// #include "STM32Relay.h"
// #define LED D10

// using namespace Relay;

// STM32Relay myRelay(STM32Relay::UART, D8, D9); 

// // void setup(){
// //     Serial.begin(115200);
// //     myRelay.begin(115200);
// //     Serial.println("Deneyap-STM32 Relay Starting...");

// //     myRelay.pinMode(STM32Relay::PB5, OUTPUT);
// //     // Serial.println("sent pinmode command...");
// // }

// // void loop(){
// //     myRelay.digitalWrite(STM32Relay::PB5, HIGH);
// //     // Serial.println("sent digital HIGH command...");
// //     delay(500);
// //     myRelay.digitalWrite(STM32Relay::PB5, LOW);
// //     // Serial.println("sent digital LOW command...");
// //     delay(500);
// // }

// void setup(){
//     Serial.begin(115200);
//     myRelay.begin(115200);

//     //physical loopback test
//     Serial.println("Physical loopback test: Connect D9 to D8 physically");
//     myRelay.sendByte(0x55);
//     delay(100);
//     uint8_t received = myRelay.recvByte(3000);
//     if(received == 0x55){
//         Serial.println("Loopback SUCCESS! Received: 0x55");
//     } else {
//         Serial.println("Loopback FAILED - check pins"); 
//         Serial.print("Received: 0x"); Serial.println(received, DEC);
//     }
// }

// void loop(){

// }

// #include <HardwareSerial.h>
// #include <Arduino.h>

// // HardwareSerial Serial2(PA9, PA10);

// void setup(){
//     Serial1.begin(115200);
//     Serial.begin(115200);
//     delay(1000);
    
//     Serial.println("Physical loopback test");

//       // Physical loopback test first
//     Serial.println("Loopback test: Connect PA9 to PA10 physically");
//     Serial1.write(0x55);
//     delay(100);
//     if(Serial1.available()){
//         uint8_t b = Serial1.read();
//         Serial.print("Loopback SUCCESS! Received: 0x");
//         Serial.println(b, HEX);
//     } else {
//         Serial.println("Loopback FAILED - check pins");
//     }
// }

// void loop(){
// }
