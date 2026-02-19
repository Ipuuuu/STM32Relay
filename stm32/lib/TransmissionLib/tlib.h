#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <stdint.h>

#include <memory>

namespace Receiver{
    
// Transmission Device Interface Class
class TDEV{
    
    public:
        // initialize the transmission device
        virtual void begin() = 0;

        // to send byte
        virtual uint8_t sendByte(uint8_t byte, uint8_t addr = 0x00) = 0;

        // to send bytes
        virtual uint8_t sendBytes(const uint8_t *bytes, size_t length, uint8_t addr = 0x00) = 0;


        // to receive bytes
        virtual uint8_t recvByte(uint8_t addr = 0x00) = 0;

        // set timeout function
        virtual void setTimeout(uint32_t timeout) = 0;

        // available function
        virtual int available() = 0;

        virtual void recover() = 0;

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

        uint8_t sendBytes(const uint8_t *bytes, size_t length, uint8_t addr = 0x00) override;

        // recv bytes through the UART device
        uint8_t recvByte(uint8_t addr = 0x00) override;

        // available overriding
        int available() override;


        // settimeout of the uart device to be used in necessary functions
        void setTimeout(uint32_t timeout) override;

        void recover() override;
};

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

        uint8_t sendBytes(const uint8_t *bytes, size_t length, uint8_t addr = 0x00) override;
    
        // RecvByte
        // @saddr : slave adress, default will be 0x00
        uint8_t recvByte(uint8_t saddr = 0x00) override;

        // available overriding
        int available() override;
        
        // set timeout for I2C device
        // should better write later with MACRO checks for 
        // WIRE_HAS_TIMEOUT
        void setTimeout(uint32_t timeout) override;

        void recover() override;
};

class I2CSlave : public TDEV{
    private:
        std::unique_ptr<TwoWire> wire;

        uint8_t rxBuffer[32]; // received buffer
        uint8_t txBuffer[32]; // transmit buffer
        volatile uint8_t rxBufferIndex, rxNext; // rxBufferIndex for receiving, rxNext for byte
        volatile uint8_t txBufferIndex, txLen;

        uint8_t addr;

        volatile uint32_t lastReceiveTime;
        volatile bool busActive;
        static const uint32_t I2C_TIMEOUT_MS = 2000;

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
        
        uint8_t sendBytes(const uint8_t *bytes, size_t length, uint8_t addr = 0x00) override;

        uint8_t recvByte(uint8_t) override;

        // available overriding
        int available() override;

        void setTimeout(uint32_t timeout) override;
        void recover() override;
        
};

// Global objects
// i am unsure about adding these or not
// because it involves assuming the use of certain transmission devices, and it may cause static initialization order issues.

//extern UARTDevice uartDev{115200, D8, D9}; // UARTDevice object for Serial1 communication
//extern I2CMaster i2cDev{}; // I2CDevice object for I2C communication
//extern I2CSlave i2cSlaveDev{I2C_ADDR_MASTER}; // I2CSlave object for I2C slave communication


};