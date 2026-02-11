#include "tlib.h"

namespace Receiver{

    // ###########################
    /* UART Device Implementation */
    // ###########################

    UARTDevice::UARTDevice(uint32_t baud, uint8_t rxPin, uint8_t txPin, HardwareSerial *uartport)
        : tout(1000), baud(baud), rxPin(rxPin), txPin(txPin), uart_port(uartport){}
    void UARTDevice::begin(){
        #ifdef ARDUINO_ARCH_STM32
        // For STM32, use the pin-defined begin
        uart_port->begin(baud);
        #else
        // For other platforms, standard begin
        uart_port->begin(baud,rxPin,txPin);
        #endif
        
        while(uart_port->available()) {
            uart_port->read();
        }
    }

    uint8_t UARTDevice::sendByte(uint8_t byte, uint8_t addr){
        uart_port->write(byte);
        uart_port->flush();
        return 0; // Success
    }

    uint8_t UARTDevice::recvByte(uint8_t addr){
        return uart_port->read();
    }

    int UARTDevice::available(){
        return uart_port->available();
    }

    void UARTDevice::setTimeout(const uint32_t timeout){
        tout = timeout;
    }


    // ###########################
    /* I2C Master Implementation */
    // ###########################

    I2CMaster::I2CMaster(TwoWire *wire)
        : wire(wire) {}

    void I2CMaster::begin(){
        // should add error checks for
        // platforms that do not support pin defined I2C
        wire->begin();
    }

    uint8_t I2CMaster::sendByte(uint8_t byte, uint8_t addr){
        wire->beginTransmission(addr);
            wire->write(byte);

            // TO-DO: error checking for endTranmission() return values
            return wire->endTransmission();
    }

    uint8_t I2CMaster::recvByte(uint8_t saddr){
        // request 1 byte
        wire->requestFrom(saddr, static_cast<uint8_t>(1));
        uint8_t requested_bytes = wire->available();
        if(!requested_bytes) return 0XFF; // error getting bytes
        return wire->read();
    }

    // available overriding (add later)
    int I2CMaster::available(){
        // not implemented yet
        return 0;
    }

    void I2CMaster::setTimeout(uint32_t timeout){
        #ifdef WIRE_HAS_TIMEOUT
        wire->setWireTimeout(timeout);
        #endif
    }

    // ###########################
    /* I2C Slave Implementation */
    // ###########################


    // Slave pointer for static ISR calls
    static I2CSlave *slave = nullptr;

    // Static ISR calls

    static void onI2CReceive(int count){
        if(!slave) return;
        while(Wire.available() && slave->rxBufferIndex < sizeof(slave->rxBuffer)){
            slave->rxBuffer[slave->rxBufferIndex++] = Wire.read();
        }
    }

    static void onI2CRequest(){
        if(!slave) return;
        for(uint8_t i = 0; i < slave->txLen; i++) Wire.write(slave->txBuffer[i]);
        slave->txLen = 0;
        slave->txBufferIndex = 0;
    }


    I2CSlave::I2CSlave(TwoWire *wire, uint8_t address)
        : wire(wire), addr(address), txLen(0), 
          txBufferIndex(0), rxBufferIndex(0), rxNext(0) {}

    // Initialize the slave connection
    // sets the address to addr
    void I2CSlave::begin(){
        slave = this;

        wire->begin(addr);

        wire->onReceive(onI2CReceive);
        wire->onRequest(onI2CRequest);
    }
    
    // this function will stack the bytes to be sent
    // until the master requests those
    uint8_t I2CSlave::sendByte(uint8_t byte, uint8_t){
        if(txLen < sizeof(txBuffer)) txBuffer[txLen++] = byte;
        return 0; // Success
    }

    // each call of this function will return
    // the next byte from the received byte stream
    uint8_t I2CSlave::recvByte(uint8_t){
        if(rxBufferIndex == 0) return 0xFF;
        uint8_t value = rxBuffer[rxNext++];
        if(rxNext == rxBufferIndex) rxNext = rxBufferIndex = 0;
        return value;
    }

    // available overriding (add later)

    int I2CSlave::available() {
        return rxBufferIndex;
    }

    void I2CSlave::setTimeout(uint32_t timeout){}
};