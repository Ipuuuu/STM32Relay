#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <stdint.h>

#include <memory>

namespace Relay{
// Transmission Device Interface Class
class TDEV{
    
    public:
        // initialize the transmission device
        virtual void begin() = 0;

        // to send bytes
        virtual uint8_t sendByte(uint8_t byte, uint8_t addr = 0x00) = 0;

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
        std::unique_ptr<HardwareSerial> uart_port; // serial port
        uint8_t txPin, rxPin; // serial pins
        
        uint32_t tout; // timeout
        uint32_t baud; // baud rate

    public:
        explicit UARTDevice(uint32_t baud, uint8_t rxPin, uint8_t txPin);

        UARTDevice(const UARTDevice&) = delete;
        UARTDevice &operator=(const UARTDevice&) = delete;
        UARTDevice(UARTDevice&&) = delete;
        UARTDevice &operator=(UARTDevice&&) = delete;

        ~UARTDevice();

        // will wrap the uart_port.begin() function, possibly with error checks
        void begin() override;

        // send bytes through the UART device
        uint8_t sendByte(uint8_t byte, uint8_t addr = 0x00) override;

        // recv bytes through the UART device
        uint8_t recvByte(uint8_t addr = 0x00) override;

        // available overriding
        int available() override;


        // settimeout of the uart device to be used in necessary functions
        void setTimeout(uint32_t timeout) override;
};



        // Extra Functionality to add later:
        // When I2C device is created, it will call the scanbus() function
        // to check hardcoded addresses and possibly query the connected
        // devices for their pinout informations if requested

// I2C Master Class
class I2CMaster : public TDEV{
    private:
        std::unique_ptr<TwoWire> wire;

    public:
        // SDA and SCL pins can be set at the constructor of the TwoWire object, so no need to set them here
        explicit I2CMaster();

        I2CMaster(const I2CMaster&) = delete;
        I2CMaster& operator=(const I2CMaster&) = delete;
        I2CMaster(I2CMaster&&) = delete;
        I2CMaster& operator=(I2CMaster&&) = delete;

        ~I2CMaster();

        // initialize device
        void begin() override;
        
        // Send bytes to the device
        uint8_t sendByte(uint8_t byte, uint8_t addr = 0x00) override;
    
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
    private:
        std::unique_ptr<TwoWire> wire;

        uint8_t rxBuffer[32]; // received buffer
        uint8_t txBuffer[32]; // transmit buffer
        volatile uint8_t rxBufferIndex, rxNext; // rxBufferIndex for receiving, rxNext for byte
        volatile uint8_t txBufferIndex, txLen;

        uint8_t addr;

        static I2CSlave *slave; // pointer to slave for static ISR calls
        static void onI2CReceive(int count); // static ISR for receiving data from master
        static void onI2CRequest(); // static ISR for sending data to master
    public:

        // SDA and SCL pins can be set at the constructor of the TwoWire object, so no need to set them here
        explicit I2CSlave(uint8_t address);

        I2CSlave(const I2CSlave&) = delete;
        I2CSlave& operator=(const I2CSlave&) = delete;
        I2CSlave(I2CSlave&&) = delete;
        I2CSlave& operator=(I2CSlave&&) = delete;

        ~I2CSlave();

        void begin() override;
        
        uint8_t sendByte(uint8_t byte, uint8_t) override;

        uint8_t recvByte(uint8_t) override;

        // available overriding
        int available() override;

        void setTimeout(uint32_t timeout) override;
        
};



};