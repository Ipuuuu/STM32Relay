#include "commapi.h"

namespace commapi{

    // ###########################
    /* UART Device Implementation */
    // ###########################

    UARTDevice::UARTDevice(HardwareSerial &serial, uint32_t baud, uint8_t rxPin, uint8_t txPin)
        : uart_port(&serial), tout(1000), baud(baud), rxPin(rxPin), txPin(txPin)
    {
    
    }

    void UARTDevice::begin(){
        #ifdef ARDUINO_ARCH_STM32
            uart_port->begin(baud);
        #else
            uart_port->begin(baud, SERIAL_8N1, rxPin, txPin); // UART1
        #endif
        
        // remove any existing bytes in the buffer
        while(uart_port->available()) {
            uart_port->read();
        }
    }

    uint8_t UARTDevice::sendByte(uint8_t byte, uint8_t addr){
        uart_port->write(byte);
        uart_port->flush();
        return 0; // Success
    }

    uint8_t UARTDevice::send(const uint8_t *bytes, uint8_t length, uint8_t addr){
        uart_port->write(bytes, length);
        uart_port->flush();
        return 0; // Success
    }

    uint8_t UARTDevice::receive(uint8_t *buf, uint8_t length, uint8_t addr){
        if(!buf || !length) return 0xFF;

        size_t bytesRead = 0;
        uint32_t startTime = millis();

        while(bytesRead < length){
            if(uart_port->available()) 
                buf[bytesRead++] = uart_port->read();
            else if((millis() - startTime) >= tout) break;
        }
        return bytesRead;
    }

    int UARTDevice::available(){
        return uart_port->available();
    }

    void UARTDevice::setTimeout(const uint32_t timeout){
        tout = timeout;
    }

    void UARTDevice::recoverIfNeeded(){
        // No specific recovery needed for UART, but could implement checks here if desired
    }


    // ###########################
    /* I2C Master Implementation */
    // ###########################

    I2CMaster::I2CMaster(TwoWire &wire) : wire(&wire) {
        // No need to create a new TwoWire instance
    }

    void I2CMaster::begin(){
        #ifdef ARDUINO_ARCH_STM32
            wire->begin();
        #else
            wire->begin(SDA, SCL); // ESP32 default pins, or pass custom ones
        #endif
    }

    uint8_t I2CMaster::sendByte(uint8_t byte, uint8_t addr){
        wire->beginTransmission(addr);
        wire->write(byte);
        return wire->endTransmission();
    }

    uint8_t I2CMaster::send(const uint8_t *bytes, uint8_t length, uint8_t addr){
        wire->beginTransmission(addr);
        wire->write(bytes, length);
        return wire->endTransmission();
    }

    uint8_t I2CMaster::receive(uint8_t *buf, uint8_t length, uint8_t addr){
        if(!buf || !length) return 0xFF;

        uint8_t received = wire->requestFrom(addr, static_cast<uint8_t>(length));
        if(received == 0) return 0xFF; // No bytes received or error

        uint8_t bytesRead = 0;
        while(wire->available() && bytesRead < length){
            buf[bytesRead++] = wire->read();
        }
        return bytesRead;
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

    void I2CMaster::recoverIfNeeded(){
        #ifdef ARDUINO_ARCH_STM32
            // Check if the bus is stuck (SDA or SCL pulled low)
            if(!digitalRead(SDA) || !digitalRead(SCL)){
                wire->end();
                delay(10);
                begin();
            }
        #endif
    }

    // ###########################
    /* I2C Slave Implementation */
    // ###########################


    // Slave pointer for static ISR calls
    I2CSlave* I2CSlave::slave = nullptr;

    // Static ISR calls

    // called on master write to slave, reads bytes into rxBuffer
    void I2CSlave::onI2CReceive(int count){
        if(!slave) return;
        slave->lastReceiveTime = millis();
        slave->busActive = true;

        slave->rxBufferIndex = 0;
        slave->rxNext = 0;
        while(slave->wire->available() && slave->rxBufferIndex < sizeof(slave->rxBuffer)){
            slave->rxBuffer[slave->rxBufferIndex++] = slave->wire->read();
        }
    }

    // called on master read from slave, writes bytes from txBuffer
    void I2CSlave::onI2CRequest(){
        if(!slave) return;
        if(slave->txBufferIndex > 0){
            slave->wire->write(slave->txBuffer, slave->txBufferIndex);
        } else {
            slave->wire->write(0xFF);
        }
    }


    I2CSlave::I2CSlave(TwoWire &wire, uint8_t address)
        : wire(&wire), addr(address), 
          txBufferIndex(0), rxBufferIndex(0), rxNext(0),
          lastReceiveTime(0), busActive(false), preparedLen(0) 
        {

        }

    // Initialize the slave connection
    // sets the address to addr
    void I2CSlave::begin(){
        slave = this;

        #ifdef ARDUINO_ARCH_STM32
            wire->begin(addr);
        #else
            wire->begin(SDA, SCL, addr); // ESP32 default pins, or pass custom ones
        #endif

        wire->onReceive(onI2CReceive);
        wire->onRequest(onI2CRequest);
    }
    
    // this function will stack the bytes to be sent
    // until the master requests those
    uint8_t I2CSlave::sendByte(uint8_t byte, uint8_t){
        if(txBufferIndex < sizeof(txBuffer)) {
            txBuffer[txBufferIndex++] = byte; 
            return 0;
        }
        return 0xFF; // Buffer full
    }

    uint8_t I2CSlave::send(const uint8_t *bytes, uint8_t length, uint8_t addr){
        noInterrupts();
        txBufferIndex = 0; // Reset buffer index before adding new data
        for(uint8_t i = 0; i < length && i < sizeof(txBuffer); i++){
            txBuffer[txBufferIndex++] = bytes[i];
        }
        txBufferIndex = (length < sizeof(txBuffer)) ? length : sizeof(txBuffer); // Ensure we don't exceed buffer size
        interrupts();
        return 0; // Success
    }

    // each call of this function will return
    // the next byte from the received byte stream

    uint8_t I2CSlave::receive(uint8_t *buf, uint8_t length, uint8_t){
        if(!buf || !length) return 0xFF;

        noInterrupts();
        uint8_t bytesRead = 0;
        while(rxNext < rxBufferIndex && bytesRead < length){
            buf[bytesRead++] = rxBuffer[rxNext++];
        }
        if(rxNext >= rxBufferIndex) rxNext = rxBufferIndex = 0;
        interrupts();
        return bytesRead;
    }

    // available overriding (add later)

    int I2CSlave::available() {
        noInterrupts();
        int count = rxBufferIndex - rxNext;
        interrupts();
        return count;
    }

    void I2CSlave::setTimeout(uint32_t timeout){}

    void I2CSlave::recoverIfNeeded(){
        #ifdef ARDUINO_ARCH_STM32
            if(!busActive) return;
            if((millis() - lastReceiveTime) < I2C_TIMEOUT_MS) return;

            // Bus was active but no data for too long - likely stuck
            wire->end();
            delay(50);
            begin();
            
            lastReceiveTime = millis();
        #endif
    }
};