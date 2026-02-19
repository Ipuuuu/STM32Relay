#pragma once

// uncomment to enable debug prints
//#define TESTMODE

// uncomment to enable UART test mode
//#define TESTMODE_UART

// uncomment to enable I2C test mode
// NOTE: When using I2C on STM32F1, TIM3 pins (PB5, PA7, PB0, PB1)
// are unavailable for PWM. Use TIM1 pins (PA8, PA9, PA10) or 
// TIM4 pins (PB8, PB9) for analogWrite instead.
#define TESTMODE_I2C

//uncomment to test LED BLINKING with digitalWrite
//#define TESTMODE_DIGITAL_WRITE

//uncomment to test ANALOG WRITE with LED FADE
#define TESTMODE_ANALOG_WRITE

//uncomment to test ANALOG READ with POTENTIOMETER
//#define TESTMODE_ANALOG_READ

//uncomment to test DIGITAL READ with BUTTON
//#define TESTMODE_DIGITAL_READ

//uncomment to enable test mode for writePPM
//#define TESTMODE_WRITEPPM


//#define I2C_ADDR_MASTER 0x00 // I2C address for the master devices
//#define I2C_ADDR_STM32 0x42 // I2C address for the slave device, can be changed as needed
//#define I2C_ADDR_RELAY 0x02 // I2C address for the relay module, can be changed as needed

