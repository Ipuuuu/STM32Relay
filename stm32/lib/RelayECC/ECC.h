#pragma once

#include <stdint.h>
#include <utility>

using bit = uint8_t;
using byte = uint8_t;

namespace Relay {
    struct CommandByte {
        enum COMMAND_TYPE : byte {
            CMD_D_W_LOW             = 0b000,
            CMD_D_W_HIGH            = 0b001,
            CMD_D_R                 = 0b010,
            CMD_A_W                 = 0b011,
            CMD_A_R                 = 0b100,
            CMD_SET_PPM             = 0b101,
            CMD_SET_PIN_MODE        = 0b110,
            CMD_EXTENDED            = 0b111
        };

        COMMAND_TYPE command : 3;
        byte ecc : 3;
        byte parity : 1;
        byte sync : 1;

        int requiresData() const {
            switch (command) {
                case CMD_A_W:
                    return 2;
                case CMD_SET_PPM:
                    return 1;
                case CMD_SET_PIN_MODE:
                    return 0;
                default:
                    return 0;
            }
        }
    };

    struct DataByte {
        byte data : 6;
        byte parity : 1;
        byte sync : 1;
    };

    struct Packet {
        CommandByte commandByte;
        DataByte port;
        DataByte data[2];

        std::pair<byte, byte> locateCorruptBit(const CommandByte& cmd, const DataByte& data) const;
        std::pair<byte, byte> locateAndCorrectError();
    };

    inline bit parity8(uint8_t byte) {
        byte ^= byte >> 4;
        byte ^= byte >> 2;
        byte ^= byte >> 1;
        return byte & 1;
    }

    inline bit parity6(uint8_t byte) {
        return parity8(byte & 0b00111111);
    }

    inline bit parity3(uint8_t nibble) {
        return parity8(nibble & 0b00000111);
    }

    inline byte oddEvenParity8(uint8_t byte) {
        byte ^= byte >> 4;
        byte ^= byte >> 2;
        return byte & 0b11;
    }

    inline byte oddEvenParity6(uint8_t byte) {
        return oddEvenParity8(byte & 0b00111111);
    }

    inline byte oddEvenParity3(uint8_t nibble) {
        return oddEvenParity8(nibble & 0b00000111);
    }

    byte calculateECC(const CommandByte& cmd, const DataByte& data); 
}