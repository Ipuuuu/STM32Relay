#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <stdint.h>

namespace Receiver{
// Transmission Device Interface Class
class TDEV{
    
    public:
        // initialize the transmission device
        virtual void begin() = 0;

        // to send bytes
        virtual void sendByte(uint8_t byte, uint8_t addr = 0x00) = 0;

        // to receive bytes
        virtual uint8_t recvByte(uint8_t addr = 0x00) = 0;

        // set timeout function
        virtual void setTimeout(uint32_t timeout) = 0;

        // available function
        virtual int available() = 0;

        virtual ~TDEV() = default;
};

// UART Base Class
class UARTDevice : public TDEV{
    private:
        uint8_t txPin, rxPin;
        
        uint32_t tout; // timeout
        uint32_t baud;

    public:
        HardwareSerial *uart_port; // serial port
        explicit UARTDevice(uint32_t baud, uint8_t rxPin, uint8_t txPin, HardwareSerial *uartport);

        UARTDevice(const UARTDevice&) = delete;
        UARTDevice &operator=(const UARTDevice&) = delete;
        ~UARTDevice() = default;

        // will wrap the uart_port.begin() function, possibly with error checks
        void begin() override;

        // send bytes through the UART device
        void sendByte(uint8_t byte, uint8_t addr = 0x00) override;

        // recv bytes through the UART device
        uint8_t recvByte(uint8_t addr = 0x00) override;

        // available overriding
        int available() override;


        // settimeout of the uart device to be used in necessary functions

        void setTimeout(uint32_t timeout) override;
};


// I2C Master Class
class I2CMaster : public TDEV{
    private:
        TwoWire *wire;

        // Ensure external pull-up resistors are used on custom 
        // SDA and SCL lines
        uint8_t sdaPin, sclPin;
    public:
        // Extra Functionality to add later:
        // When I2C device is created, it will call the scanbus() function
        // to check hardcoded addresses and possibly query the connected
        // devices for their pinout informations if requested
        explicit I2CMaster(TwoWire *wire, uint8_t sdaPin, uint8_t sclPin);

        I2CMaster(const I2CMaster&) = delete;
        I2CMaster& operator=(const I2CMaster&) = delete;
        ~I2CMaster() = default;

        // initialize device
        void begin() override;
        
        // Send bytes to the device
        void sendByte(uint8_t byte, uint8_t addr = 0x00) override;
    
        // RecvByte
        // @saddr : slave adress, default will be 0x00
        uint8_t recvByte(uint8_t saddr = 0x00) override;

        // available overriding
        int available() override;
        
        // set timeout for I2C device
        // should better write later with MACRO checks for 
        // WIRE_HAS_TIMEOUT
        void setTimeout(uint32_t timeout) override;

        // A function like setSlave can be written to set the
        // slave address for multiple calls, but for now
        // each sendByte and recvByte will have the address parameter
};

class I2CSlave : public TDEV{
    public:

        TwoWire *wire;

        // Ensure external pull-up resistors are used on custom 
        // SDA and SCL lines
        uint8_t sdaPin, sclPin;
    
        uint8_t rxBuffer[32]; // received buffer
        uint8_t txBuffer[32]; // transmit buffer
        volatile uint8_t rxBufferIndex, rxNext; // rxBufferIndex for receiving, rxNext for byte
        volatile uint8_t txBufferIndex, txLen;

        uint8_t addr;

        explicit I2CSlave(TwoWire *wire, uint8_t address, uint8_t sdaPin, uint8_t sclPin);

        I2CSlave(const I2CSlave&) = delete;
        I2CSlave& operator=(const I2CSlave&) = delete;
        ~I2CSlave() = default;

        void begin() override;
        
        void sendByte(uint8_t byte, uint8_t) override;

        uint8_t recvByte(uint8_t) override;

        // available overriding
        int available() override;

        void setTimeout(uint32_t timeout) override;
        
};



};