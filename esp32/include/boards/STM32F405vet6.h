#pragma once
#include "../Pin.h"

namespace Relay {
    // STM32F407VET6 Pin definitions (based on Arduino variant_BLACK_F407VX.h)
    
    // Right Side
    const Pin PE1(0);
    const Pin PE0(1);
    const Pin PB9(2);
    const Pin PB8(3);
    const Pin PB7(4);
    const Pin PB6(5);
    const Pin PB5(6);
    const Pin PB3(7);
    const Pin PD7(8);
    const Pin PD6(9);
    const Pin PD5(10);
    const Pin PD4(11);
    const Pin PD3(12);
    const Pin PD2(13);
    const Pin PD1(14);
    const Pin PD0(15);
    const Pin PC12(16);
    const Pin PC11(17);
    const Pin PC10(18);
    const Pin PA15(19);
    const Pin PA12(20);  // USB_DP
    const Pin PA11(21);  // USB_DM
    const Pin PA10(22);
    const Pin PA9(23);
    const Pin PA8(24);
    const Pin PC9(25);
    const Pin PC8(26);
    const Pin PC7(27);
    const Pin PC6(28);
    const Pin PD15(29);
    const Pin PD14(30);
    const Pin PD13(31);
    const Pin PD12(32);
    const Pin PD11(33);
    const Pin PD10(34);
    const Pin PD9(35);
    const Pin PD8(36);
    const Pin PB15(37);
    
    // Left Side
    const Pin PE2(38);
    const Pin PE3(39);
    const Pin PE4(40);   // BUT K0
    const Pin PE5(41);   // BUT K1
    const Pin PE6(42);
    const Pin PC13(43);
    const Pin PC0(44);   // PIN_A0
    const Pin PC1(45);   // PIN_A1
    const Pin PC2(46);   // PIN_A2
    const Pin PC3(47);   // PIN_A3
    const Pin PA0(48);   // PIN_A4, WK_UP: BUT K_UP
    const Pin PA1(49);   // PIN_A5
    const Pin PA2(50);   // PIN_A6
    const Pin PA3(51);   // PIN_A7
    const Pin PA4(52);   // PIN_A8
    const Pin PA5(53);   // PIN_A9
    const Pin PC4(54);   // PIN_A10
    const Pin PC5(55);   // PIN_A11
    const Pin PB0(56);   // PIN_A12
    const Pin PB1(57);   // PIN_A13
    const Pin PA6(58);   // PIN_A14, LED D2
    const Pin PA7(59);   // PIN_A15, LED D3 (active LOW)
    const Pin PE7(60);
    const Pin PE8(61);
    const Pin PE9(62);
    const Pin PE10(63);
    const Pin PE11(64);
    const Pin PE12(65);
    const Pin PE13(66);
    const Pin PE14(67);
    const Pin PE15(68);
    const Pin PB10(69);
    const Pin PB11(70);
    const Pin PB12(71);
    const Pin PB13(72);
    const Pin PB14(73);
    const Pin PB4(74);

    // Named aliases for LEDs and buttons
    const Pin& LED_D2 = PA6;
    const Pin& LED_D3 = PA7;
    const Pin& LED_GREEN = LED_D2;
    const Pin& LED_BUILTIN = LED_D2;
    
    const Pin& BTN_K_UP = PA0;
    const Pin& BTN_K0 = PE4;
    const Pin& BTN_K1 = PE3;
    const Pin& USER_BTN = BTN_K0;

    // SPI pins (NRF24 connector & W25Q16 on-board flash)
    const Pin& PIN_SPI_SS = PB7;    // NRF24 connector
    const Pin& PIN_SPI_SS1 = PB0;   // W25Q16 (on board flash)
    const Pin& PIN_SPI_MOSI = PB5;
    const Pin& PIN_SPI_MISO = PB4;
    const Pin& PIN_SPI_SCK = PB3;

    // I2C pins
    const Pin& PIN_WIRE_SDA = PB9;
    const Pin& PIN_WIRE_SCL = PB8;

    // UART pins
    const Pin& PIN_SERIAL_RX = PA10;
    const Pin& PIN_SERIAL_TX = PA9;

    // Alternate pin functions
    const Pin PA0_ALT1(48 | (1 << 8));
    const Pin PA0_ALT2(48 | (2 << 8));
    const Pin PA1_ALT1(49 | (1 << 8));
    const Pin PA1_ALT2(49 | (2 << 8));
    const Pin PA2_ALT1(50 | (1 << 8));
    const Pin PA2_ALT2(50 | (2 << 8));
    const Pin PA3_ALT1(51 | (1 << 8));
    const Pin PA3_ALT2(51 | (2 << 8));
    const Pin PA4_ALT1(52 | (1 << 8));
    const Pin PA5_ALT1(53 | (1 << 8));
    const Pin PA6_ALT1(58 | (1 << 8));
    const Pin PA7_ALT1(59 | (1 << 8));
    const Pin PA7_ALT2(59 | (2 << 8));
    const Pin PA7_ALT3(59 | (3 << 8));
    const Pin PA15_ALT1(19 | (1 << 8));
    const Pin PB0_ALT1(56 | (1 << 8));
    const Pin PB0_ALT2(56 | (2 << 8));
    const Pin PB1_ALT1(57 | (1 << 8));
    const Pin PB1_ALT2(57 | (2 << 8));
    const Pin PB3_ALT1(7 | (1 << 8));
    const Pin PB4_ALT1(74 | (1 << 8));
    const Pin PB5_ALT1(6 | (1 << 8));
    const Pin PB8_ALT1(3 | (1 << 8));
    const Pin PB9_ALT1(2 | (1 << 8));
    const Pin PB14_ALT1(73 | (1 << 8));
    const Pin PB14_ALT2(73 | (2 << 8));
    const Pin PB15_ALT1(37 | (1 << 8));
    const Pin PB15_ALT2(37 | (2 << 8));
    const Pin PC0_ALT1(44 | (1 << 8));
    const Pin PC0_ALT2(44 | (2 << 8));
    const Pin PC1_ALT1(45 | (1 << 8));
    const Pin PC1_ALT2(45 | (2 << 8));
    const Pin PC2_ALT1(46 | (1 << 8));
    const Pin PC2_ALT2(46 | (2 << 8));
    const Pin PC3_ALT1(47 | (1 << 8));
    const Pin PC3_ALT2(47 | (2 << 8));
    const Pin PC4_ALT1(54 | (1 << 8));
    const Pin PC5_ALT1(55 | (1 << 8));
    const Pin PC6_ALT1(28 | (1 << 8));
    const Pin PC7_ALT1(27 | (1 << 8));
    const Pin PC8_ALT1(26 | (1 << 8));
    const Pin PC9_ALT1(25 | (1 << 8));
    const Pin PC10_ALT1(18 | (1 << 8));
    const Pin PC11_ALT1(17 | (1 << 8));
}