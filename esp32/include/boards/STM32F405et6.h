#pragma once
#include "../Pin.h"

namespace Relay {
    // STM32F407VET6 Pin definitions (LQFP100 package)
    
    // Port A
    const Pin PA0(0);
    const Pin PA1(1);   // OSCIN (Ethernet RETCLK)
    const Pin PA2(2);   // MDIO (Ethernet)
    const Pin PA3(3);
    const Pin PA4(4);
    const Pin PA5(5);
    const Pin PA6(6);   // LED D2
    const Pin PA7(7);   // LED D3, CRS (Ethernet)
    const Pin PA8(8);
    const Pin PA9(9);
    const Pin PA10(10);
    const Pin PA11(11); // USB DM, CAN1_RX
    const Pin PA12(12); // USB DP, CAN1_TX
    const Pin PA13(13); // SWDIO
    const Pin PA14(14); // SWCLK
    const Pin PA15(15);

    // Port B
    const Pin PB0(16);
    const Pin PB1(17);
    const Pin PB2(18);  // BOOT1
    const Pin PB3(19);
    const Pin PB4(20);
    const Pin PB5(21);  // CAN2_RX
    const Pin PB6(22);  // CAN2_TX
    const Pin PB7(23);
    const Pin PB8(24);  // I2C_SCL, CAN1_RX
    const Pin PB9(25);  // I2C_SDA, CAN1_TX
    const Pin PB10(26); // SPI_SCK
    const Pin PB11(27); // TX_EN (Ethernet)
    const Pin PB12(28); // TX0 (Ethernet), CAN2_RX
    const Pin PB13(29); // TX1 (Ethernet), CAN2_TX
    const Pin PB14(30);
    const Pin PB15(31);

    // Port C
    const Pin PC0(32);
    const Pin PC1(33);  // MDC (Ethernet)
    const Pin PC2(34);  // SPI_MISO
    const Pin PC3(35);  // SPI_MOSI
    const Pin PC4(36);  // RX0 (Ethernet)
    const Pin PC5(37);  // RX1 (Ethernet)
    const Pin PC6(38);  // USBTX (USART6_TX)
    const Pin PC7(39);  // USBRX (USART6_RX)
    const Pin PC8(40);  // SDIO_D0
    const Pin PC9(41);  // SDIO_D1
    const Pin PC10(42); // SDIO_D2, SPI3_SCK
    const Pin PC11(43); // SDIO_D3, SPI3_MISO
    const Pin PC12(44); // SDIO_CLK, SPI3_MOSI
    const Pin PC13(45);
    const Pin PC14(46);
    const Pin PC15(47);

    // Port D
    const Pin PD0(48);  // CAN1_RX
    const Pin PD1(49);  // CAN1_TX
    const Pin PD2(50);  // SDIO_CMD
    const Pin PD3(51);
    const Pin PD4(52);
    const Pin PD5(53);
    const Pin PD6(54);
    const Pin PD7(55);
    const Pin PD8(56);
    const Pin PD9(57);
    const Pin PD10(58);
    const Pin PD11(59);
    const Pin PD12(60);
    const Pin PD13(61);
    const Pin PD14(62);
    const Pin PD15(63);

    // Port E
    const Pin PE0(64);
    const Pin PE1(65);
    const Pin PE2(66);
    const Pin PE3(67);  // K1 button, SPI_CS
    const Pin PE4(68);  // K0 button
    const Pin PE5(69);
    const Pin PE6(70);
    const Pin PE7(71);
    const Pin PE8(72);
    const Pin PE9(73);
    const Pin PE10(74);
    const Pin PE11(75);
    const Pin PE12(76);
    const Pin PE13(77);
    const Pin PE14(78);
    const Pin PE15(79);

    // Named aliases for common functions
    const Pin& LED1 = PA6;       // LED D2
    const Pin& LED2 = PA7;       // LED D3
    const Pin& USER_BUTTON_K0 = PE4;
    const Pin& USER_BUTTON_K1 = PE3;
    const Pin& WAKEUP_BUTTON = PA0;

    // USART aliases
    const Pin& USBTX = PC6;
    const Pin& USBRX = PC7;

    // I2C aliases
    const Pin& I2C_SCL = PB8;
    const Pin& I2C_SDA = PB9;

    // SPI aliases
    const Pin& SPI_MOSI = PC3;
    const Pin& SPI_MISO = PC2;
    const Pin& SPI_SCK = PB10;
    const Pin& SPI_CS = PE3;

    // Alternate pin functions (if needed for your Pin class)
    // CAN1
    const Pin PA11_CAN1_RX(11 | (1 << 8));
    const Pin PA12_CAN1_TX(12 | (1 << 8));
    const Pin PB8_CAN1_RX(24 | (1 << 8));
    const Pin PB9_CAN1_TX(25 | (1 << 8));
    const Pin PD0_CAN1_RX(48 | (1 << 8));
    const Pin PD1_CAN1_TX(49 | (1 << 8));

    // CAN2
    const Pin PB5_CAN2_RX(21 | (1 << 8));
    const Pin PB6_CAN2_TX(22 | (1 << 8));
    const Pin PB12_CAN2_RX(28 | (1 << 8));
    const Pin PB13_CAN2_TX(29 | (1 << 8));
}