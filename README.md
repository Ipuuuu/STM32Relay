# STM32Relay

A flexible, multi-microcontroller relay control library that enables seamless communication and relay control between different microcontroller platforms (STM32, ESP32, Arduino, etc.) using UART with optional I2C support and robust error detection/correction.

## 📋 Project Overview

**STM32Relay** is an inter-microcontroller communication library designed to turn any microcontroller into a relay bridge. This means you can:

- Use an **STM32F1/F2/F4 as a master** controlling relays on **ESP32, Arduino, or other STM32 variants**
- Use an **ESP32 as a master** controlling relays on **STM32, Arduino, or other boards**
- Use an **Arduino as a master** controlling relays on **ESP32, STM32, or other platforms**
- **Bi-directional control** - any board can act as master or slave depending on your application

The library includes:
- **Board-agnostic design** with specific configuration files for each supported microcontroller
- **UART-based communication** (I2C support coming soon)
- **ECC (Error Correction Code)** implementation for reliable single-bit error detection and correction
- **Automatic burst error handling** with data retransmission

## ✨ Key Features

- **Multi-Microcontroller Support** - Works with STM32 (F1, F2, F4), ESP32, Arduino, and more
- **Board Configuration System** - Easy board-specific pin mapping and settings
- **Master/Slave Architecture** - Flexible role assignment for your application needs
- **UART Communication** - Reliable serial communication between devices
- **I2C Support** - Framework ready for I2C protocol implementation
- **ECC Error Detection & Correction** - Hamming code implementation for single-bit error recovery
- **Automatic Retransmission** - Burst errors trigger automatic data resend
- **PlatformIO Compatible** - Seamless project setup across platforms
- **Well-Structured Code** - Modular design with clear separation of concerns


## 🚀 Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/install) installed and configured
- Python 3.5 or higher
- Git (for cloning the repository)
- Serial communication cables (USB to UART or direct connection based on your setup)

### Installation

#### 1. Clone the Repository

```bash
git clone https://github.com/Ipuuuu/STM32Relay.git
cd STM32Relay 
```
#### 2. Choose Your Role
Decide which microcontroller will be the master (controls relays) and which will be the slave (responds to relay commands):

Master: Sends relay control commands via UART/I2C
Slave: Receives commands and controls relays on its GPIO pins

#### 📚 Usage Guide
Understanding the Architecture
The STM32Relay library uses a Master-Slave communication model:
```bash
┌─────────────────────┐                ┌──────────────────────┐
│   MASTER            │  UART/I2C      │    SLAVE             │
│  (STM32/ESP32/etc)  │◄─────────────► │  (STM32/ESP32/etc)   │
│                     │                │                      │
│  - Sends commands   │  UART RX/TX    │  - Receives commands │
│  - Controls relays  │  or I2C SCL/SDA│  - Controls relays   │
│  - Monitors status  │                │  - Sends status      │
└─────────────────────┘                └──────────────────────┘
```

#### 🔧 Hardware Setup
Wiring: STM32F103 to ESP32 (UART)
```bash
STM32F103          ESP32
   PA9   (TX) ────► RX (GPIO16)
   PA10  (RX) ◄──── TX (GPIO17)
   GND ────────────── GND
```

Wiring: Arduino to ESP32 (UART)
```bash
Arduino            ESP32
   TX (D1) ────► RX (GPIO16)
   RX (D0) ◄──── TX (GPIO17)
   GND ────────────── GND
```

Relay Connection
```bash
GPIO Pin ──┐
           ├─►[Relay Module]──► Output
      GND ─┘
```

#### 🆘 Troubleshooting
##### Communication Not Working
- Check UART connections - Verify TX/RX wires are crossed correctly
- Verify baud rates - Both devices must use the same baud rate (typically 115200)
- Check power supply - Ensure stable power to both microcontrollers
- Monitor serial output - Use platformio device monitor to debug

##### Errors Being Detected Too Frequently
- Check cable quality - Use shielded cables for UART
- Add pull-up resistors - Add 10kΩ resistors on TX/RX lines
- Reduce baud rate - Try 9600 or 38400 for longer cables
