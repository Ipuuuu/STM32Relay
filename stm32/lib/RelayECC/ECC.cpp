#include "ECC.h"

namespace Relay {

    byte calculateECC(const CommandByte& cmd, const DataByte& data) {
        byte ecc = 0;

        byte byte1 = (static_cast<byte>(oddEvenParity3(cmd.command)) << 6) | (cmd.command);
        byte byte2 = (static_cast<byte>(oddEvenParity6(data.data)) << 6) | (data.data);

        bool p1, p2, p3;
        p1 = parity8(byte1 & 0b11110000) ^ parity8(byte2 & 0b11110000);
        p2 = parity8(byte1 & 0b11001100) ^ parity8(byte2 & 0b11001100);
        p3 = parity8(byte1 & 0b00101010) ^ parity8(byte2 & 0b00101010);

        ecc |= (p1 << 2);
        ecc |= (p2 << 1);
        ecc |= p3;

        return ecc;
    }

    std::pair<byte, byte> Relay::Packet::locateCorruptBit(const CommandByte& cmd, const DataByte& data) const {
        auto receivedECC = cmd.ecc;
        auto calculatedECC = calculateECC(cmd, data);
        byte errorPattern = receivedECC ^ calculatedECC;

        if (errorPattern == 0) {
            return {0, 0}; // No error
        }

        // Check parities to determine if error is in command or data byte
        byte errorByte = 0;
        if(parity3(static_cast<uint8_t>(cmd.command)) != cmd.parity) {
            errorByte = 1; // command byte
        } else if(parity6(data.data) != data.parity) {
            errorByte = 2; // data byte
        } else {
            return {0, 0}; // no error detected
        }

        byte errorBit = 0b00001111 ^ (((errorPattern & 0b100) == 0) * 0b11111111);
        errorBit &= 0b00110011 ^ (((errorPattern & 0b010) == 0) * 0b11111111);
        errorBit &= 0b00101010 ^ (((errorPattern & 0b1) == 0) * 0b00111111);

        // //Find the bit position (count trailing zeros or find first set bit)
        // byte bitPosition = 0;
        // for(int i = 0; i < 8; ++i) {
        //     if(errorBit & (1 << i)) {
        //         bitPosition = i;
        //         break;
        //     }
        // }

        // return {errorByte,bitPosition};

        // Find bit position
        byte bitPosition = 0;
        #ifdef __GNUC__
            bitPosition = __builtin_ctz(errorBit);
        #else
            for(int i = 0; i < 8; ++i) {
                if(errorBit & (1 << i)) {
                    bitPosition = i;
                    break;
                }
            }
        #endif

        return {errorByte, bitPosition};
    }

    std::pair<byte, byte> Relay::Packet::locateAndCorrectError() {
        int dataBytes = commandByte.requiresData();
        
        // Check command byte against port
        auto result = locateCorruptBit(commandByte, port);
        if (result.first == 1) {
            // Error in command byte
            commandByte.command = static_cast<CommandByte::COMMAND_TYPE>(
                commandByte.command ^ (1 << result.second)
            );
            commandByte.parity = parity3(static_cast<uint8_t>(commandByte.command));
            return {1, result.second};
        } else if (result.first == 2) {
            // Error in port byte
            port.data ^= (1 << result.second);
            port.parity = parity6(port.data);
            return {2, result.second};
        }
        
        // Check data[0] if needed
        if (dataBytes >= 1) {
            result = locateCorruptBit(commandByte, data[0]);
            if (result.first == 1) {
                commandByte.command = static_cast<CommandByte::COMMAND_TYPE>(
                    commandByte.command ^ (1 << result.second)
                );
                commandByte.parity = parity3(static_cast<uint8_t>(commandByte.command));
                return {1, result.second};
            } else if (result.first == 2) {
                data[0].data ^= (1 << result.second);
                data[0].parity = parity6(data[0].data);
                return {3, result.second};
            }
        }
        
        // Check data[1] if needed
        if (dataBytes >= 2) {
            result = locateCorruptBit(commandByte, data[1]);
            if (result.first == 1) {
                commandByte.command = static_cast<CommandByte::COMMAND_TYPE>(
                    commandByte.command ^ (1 << result.second)
                );
                commandByte.parity = parity3(static_cast<uint8_t>(commandByte.command));
                return {1, result.second};
            } else if (result.first == 2) {
                data[1].data ^= (1 << result.second);
                data[1].parity = parity6(data[1].data);
                return {4, result.second};
            }
        }
        
        return {0, 0};
    }
}