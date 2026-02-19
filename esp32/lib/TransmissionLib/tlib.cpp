#include "tlib.h"

namespace Relay{

    // ###########################
    /* UART Device Implementation */
    // ###########################

    UARTDevice::UARTDevice(uint32_t baud, uint8_t rxPin, uint8_t txPin)
        : tout(1000), baud(baud), rxPin(rxPin), txPin(txPin){
    
        #ifdef ARDUINO_ARCH_STM32
            uart_port = std::make_unique<HardwareSerial>(rxPin, txPin);
        #else
            uart_port = std::make_unique<HardwareSerial>(1); // UART1
        #endif
    }

    UARTDevice::~UARTDevice(){}

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

    uint8_t UARTDevice::sendBytes(const uint8_t *bytes, size_t length, uint8_t addr){
        uart_port->write(bytes, length);
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

    void UARTDevice::recoverIfNeeded(){
        // No specific recovery needed for UART, but could implement checks here if desired
    }


    // ###########################
    /* I2C Master Implementation */
    // ###########################

    I2CMaster::I2CMaster() {
        #ifdef ARDUINO_ARCH_STM32
            wire = std::make_unique<TwoWire>();
        #else
            wire = std::make_unique<TwoWire>(0); // I2C0
        #endif
    }

    I2CMaster::~I2CMaster() {}

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

    uint8_t I2CMaster::sendBytes(const uint8_t *bytes, size_t length, uint8_t addr){
        wire->beginTransmission(addr);
        wire->write(bytes, length);

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
        while(slave->wire->available()){
            uint8_t x = slave->wire->read();
            if(slave->rxBufferIndex < sizeof(slave->rxBuffer)) slave->rxBuffer[slave->rxBufferIndex++] = x;
        }
    }

    // called on master read from slave, writes bytes from txBuffer
    void I2CSlave::onI2CRequest(){
        if(!slave) return;
        if(slave->txLen == 0) {
            slave->wire->write(0xFF); // send dummy byte to prevent clock stretch hang
        } else {
            for(uint8_t i = 0; i < slave->txLen; i++) slave->wire->write(slave->txBuffer[i]);
        }
        slave->txLen = 0;
        slave->txBufferIndex = 0;
    }


    I2CSlave::I2CSlave(uint8_t address)
        : addr(address), txLen(0), 
          txBufferIndex(0), rxBufferIndex(0), rxNext(0),
          lastReceiveTime(0), busActive(false) {
            #ifdef ARDUINO_ARCH_STM32
                wire = std::make_unique<TwoWire>();
            #else
                wire = std::make_unique<TwoWire>(0); // I2C0
            #endif
        }

    I2CSlave::~I2CSlave() {}

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
        if(txLen < sizeof(txBuffer)) txBuffer[txLen++] = byte;
        return 0; // Success
    }

    uint8_t I2CSlave::sendBytes(const uint8_t *bytes, size_t length, uint8_t addr){
        for(size_t i = 0; i < length; i++){
            if(txLen < sizeof(txBuffer)) txBuffer[txLen++] = bytes[i];
            else break; // Buffer full, stop adding more bytes
        }
        return 0; // Success
    }

    // each call of this function will return
    // the next byte from the received byte stream
    uint8_t I2CSlave::recvByte(uint8_t){
        noInterrupts();
        if(rxNext >= rxBufferIndex){
            rxNext = rxBufferIndex = 0;
            interrupts();
            return 0xFF;
        }
        
        uint8_t value = rxBuffer[rxNext++];
        if(rxNext >= rxBufferIndex) rxNext = rxBufferIndex = 0;
        interrupts();
        return value;
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