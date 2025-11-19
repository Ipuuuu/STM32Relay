#pragma once
#include "../Pin.h"

namespace Relay{
    //STM32 Pin definitions
    // Left side pins
    const Pin PB9(0);
    const Pin PB8(1);
    const Pin PB7(2);
    const Pin PB6(3);
    const Pin PB5(4);
    const Pin PB4(5);
    const Pin PB3(6);
    const Pin PA15(7);
    const Pin PA12(8);   // USB DP
    const Pin PA11(9);   // USB DM
    const Pin PA10(10);
    const Pin PA9(11);
    const Pin PA8(12);
    const Pin PB15(13);
    const Pin PB14(14);
    const Pin PB13(15);
    const Pin PB12(16);  // LED Blackpill

    // Right side pins
    const Pin PC13(17);  // LED Bluepill
    const Pin PC14(18);
    const Pin PC15(19);

    // Analog pins (Arduino-style)
    const Pin PA0(20);  // PIN_A0
    const Pin PA1(21);  // PIN_A1
    const Pin PA2(22);  // PIN_A2
    const Pin PA3(23);  // PIN_A3
    const Pin PA4(24);  // PIN_A4
    const Pin PA5(25);  // PIN_A5
    const Pin PA6(26);  // PIN_A6
    const Pin PA7(27);  // PIN_A7
    const Pin PB0(28);  // PIN_A8
    const Pin PB1(29);  // PIN_A9

    const Pin PB10(30);
    const Pin PB11(31);

    // Other pins
    const Pin PB2(32);   // BOOT1
    const Pin PA13(33);  // SWDIO
    const Pin PA14(34);  // SWCLK

    // Alternate pins (optional, if you want)
    const Pin PA0_ALT1(20 | (1 << 8));
    const Pin PA1_ALT1(21 | (1 << 8));
    const Pin PA2_ALT1(22 | (1 << 8));
    const Pin PA3_ALT1(23 | (1 << 8));
    const Pin PA4_ALT1(24 | (1 << 8));
    const Pin PA5_ALT1(25 | (1 << 8));
    const Pin PA6_ALT1(26 | (1 << 8));
    const Pin PA7_ALT1(27 | (1 << 8));
    const Pin PA8_ALT1(12 | (1 << 8));   // PA8 base number
    const Pin PA9_ALT1(11 | (1 << 8));   // PA9 base number
    const Pin PA10_ALT1(10 | (1 << 8));
    const Pin PA11_ALT1(9 | (1 << 8));
    const Pin PA15_ALT1(7 | (1 << 8));
    const Pin PB0_ALT1(28 | (1 << 8));
    const Pin PB0_ALT2(28 | (2 << 8));
    const Pin PB1_ALT1(29 | (1 << 8));
    const Pin PB1_ALT2(29 | (2 << 8));
    const Pin PB3_ALT1(6 | (1 << 8));
    const Pin PB10_ALT1(30 | (1 << 8));
    const Pin PB11_ALT1(31 | (1 << 8));
    const Pin PB13_ALT1(15 | (1 << 8));
    const Pin PB14_ALT1(14 | (1 << 8));

}