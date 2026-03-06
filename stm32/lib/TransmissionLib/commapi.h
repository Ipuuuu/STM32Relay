#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <stdint.h>

#include <memory>

namespace commapi{
    
// Communication Interface Class
class ICOMM{

    protected:
        // to send a single byte
        virtual uint8_t sendByte(uint8_t byte, uint8_t addr = 0x00) = 0;
    
    public:
        // initialize the transmission device
        virtual void begin() = 0;

        // to send bytes
        virtual uint8_t send(const uint8_t *bytes, uint8_t length, uint8_t addr = 0x00) = 0;
        virtual uint8_t send(uint8_t byte, uint8_t addr = 0x00){return send(&byte, 1, addr);}

        // to receive bytes
        virtual uint8_t receive(uint8_t *buf, uint8_t length, uint8_t addr = 0x00) = 0;

        // set timeout function
        virtual void setTimeout(uint32_t timeout) = 0;

        // available function
        virtual int available() = 0;

        virtual void recoverIfNeeded() = 0;

        virtual ~ICOMM() = default;
};

// UART Base Class
class UARTDevice : public ICOMM{
    private:
        HardwareSerial *uart_port; // serial port
        uint8_t txPin, rxPin; // serial pins
        
        uint32_t tout; // timeout
        uint32_t baud; // baud rate

        // send bytes through the UART device
        uint8_t sendByte(uint8_t byte, uint8_t addr = 0x00) override;

    public:
        explicit UARTDevice(HardwareSerial &serial, uint32_t baud, uint8_t rxPin, uint8_t txPin);

        UARTDevice(const UARTDevice&) = delete;
        UARTDevice &operator=(const UARTDevice&) = delete;
        UARTDevice(UARTDevice&&) = delete;
        UARTDevice &operator=(UARTDevice&&) = delete;

        ~UARTDevice() = default;

        // will wrap the uart_port.begin() function, possibly with error checks
        void begin() override;

        uint8_t send(const uint8_t *bytes, uint8_t length, uint8_t addr = 0x00) override;

        // recv bytes through the UART device
        uint8_t receive(uint8_t *buf, uint8_t length, uint8_t addr = 0x00) override;

        // available overriding
        int available() override;


        // settimeout of the uart device to be used in necessary functions
        void setTimeout(uint32_t timeout) override;

        void recoverIfNeeded();
};

// I2C Master Class
class I2CMaster : public ICOMM{
    private:
        TwoWire *wire;
        uint8_t sdaPin, sclPin;

        // send bytes through the UART device
        uint8_t sendByte(uint8_t byte, uint8_t addr = 0x00) override;

    public:
        // SDA and SCL pins can be set at the constructor of the TwoWire object, so no need to set them here
        explicit I2CMaster(TwoWire &wire, uint8_t sdaPin, uint8_t sclPin);

        I2CMaster(const I2CMaster&) = delete;
        I2CMaster& operator=(const I2CMaster&) = delete;
        I2CMaster(I2CMaster&&) = delete;
        I2CMaster& operator=(I2CMaster&&) = delete;

        ~I2CMaster() = default;

        // initialize device
        void begin() override;

        uint8_t send(const uint8_t *bytes, uint8_t length, uint8_t addr = 0x00) override;
    
        // RecvByte
        // @saddr : slave adress, default will be 0x00
        uint8_t receive(uint8_t *buf, uint8_t length, uint8_t addr = 0x00) override;

        // available overriding
        int available() override;
        
        // set timeout for I2C device
        // should better write later with MACRO checks for 
        // WIRE_HAS_TIMEOUT
        void setTimeout(uint32_t timeout) override;

        void recoverIfNeeded();
};

class I2CSlave : public ICOMM{
    private:
        TwoWire *wire;
        uint8_t sdaPin, sclPin;

        uint8_t rxBuffer[32]; // received buffer
        uint8_t txBuffer[32]; // transmit buffer
        volatile uint8_t rxBufferIndex, rxNext; // rxBufferIndex for receiving, rxNext for byte
        volatile uint8_t txBufferIndex;

        uint8_t addr;

        volatile uint32_t lastReceiveTime;
        volatile bool busActive;
        static const uint32_t I2C_TIMEOUT_MS = 2000;

        static I2CSlave *slave; // pointer to slave for static ISR calls
        static void onI2CReceive(int count); // static ISR for receiving data from master
        static void onI2CRequest(); // static ISR for sending data to master

        // send bytes through the UART device
        uint8_t sendByte(uint8_t byte, uint8_t addr = 0x00) override;
    public:

        // SDA and SCL pins can be set at the constructor of the TwoWire object, so no need to set them here
        explicit I2CSlave(TwoWire &wire, uint8_t sdaPin, uint8_t sclPin, uint8_t address);

        I2CSlave(const I2CSlave&) = delete;
        I2CSlave& operator=(const I2CSlave&) = delete;
        I2CSlave(I2CSlave&&) = delete;
        I2CSlave& operator=(I2CSlave&&) = delete;

        ~I2CSlave() = default;

        void begin() override;
        
        uint8_t send(const uint8_t *bytes, uint8_t length, uint8_t addr = 0x00) override;

        uint8_t receive(uint8_t *buf, uint8_t length, uint8_t addr = 0x00) override;

        // available overriding
        int available() override;

        void setTimeout(uint32_t timeout) override;
        void recoverIfNeeded();
        
};

// Global objects
// i am not sure about adding these or not
// because it involves assuming the use of certain transmission devices, and it may cause static initialization order issues.

//extern UARTDevice uartDev{115200, D8, D9}; // UARTDevice object for Serial1 communication
//extern I2CMaster i2cDev{}; // I2CDevice object for I2C communication
//extern I2CSlave i2cSlaveDev{I2C_ADDR_MASTER}; // I2CSlave object for I2C slave communication


};