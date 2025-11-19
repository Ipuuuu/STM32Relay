#pragma once
#include "../Pin.h"

namespace Relay {

    // --- Digital Pins D0–D13 ---
    const Pin D0(0);     // PD0
    const Pin D1(1);     // PD1
    const Pin D2(2);     // PD2
    const Pin D3(3);     // PD3 (PWM)
    const Pin D4(4);     // PD4
    const Pin D5(5);     // PD5 (PWM)
    const Pin D6(6);     // PD6 (PWM)
    const Pin D7(7);     // PD7
    const Pin D8(8);     // PB0
    const Pin D9(9);     // PB1 (PWM)
    const Pin D10(10);   // PB2 (PWM) SS
    const Pin D11(11);   // PB3 (PWM) MOSI
    const Pin D12(12);   // PB4 MISO
    const Pin D13(13);   // PB5 SCK  (LED_BUILTIN)

    // --- Analog Pins A0–A5 (mapped to digital 14–19) ---
    const Pin A0(14);    // PC0
    const Pin A1(15);    // PC1
    const Pin A2(16);    // PC2
    const Pin A3(17);    // PC3
    const Pin A4(18);    // PC4 (SDA)
    const Pin A5(19);    // PC5 (SCL)

    // --- Optional A6, A7 (only on DIP Nano, not on Uno) ---
    // Include these only if you want broader AVR support:
    // const Pin A6(20);  
    // const Pin A7(21);

    // --- SPI Pins ---
    const Pin SS(10);
    const Pin MOSI(11);
    const Pin MISO(12);
    const Pin SCK(13);

    // --- I2C Pins ---
    const Pin SDA(18);   // A4
    const Pin SCL(19);   // A5

    // Built-in LED
    const Pin LED_BUILTIN(13);
}
