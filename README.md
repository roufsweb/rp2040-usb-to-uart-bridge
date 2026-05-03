# RP2040 USB-to-TTL Serial Bridge

A high-performance USB-to-UART bridge firmware for RP2040-based boards. Specifically optimized for boards with an onboard WS2812 RGB LED (like the VCC-GND style boards).

## Features
- **USB CDC to UART**: Provides a standard serial port on your computer.
- **DTR/RTS Support**: Supports hardware flow control and auto-reset for ESP32/ESP8266 flashing.
- **Visual Status (RGB LED)**:
  - 🔴 **Red**: USB Not Ready / Disconnected.
  - 🟢 **Dim Green**: Idle / Ready.
  - 🔵 **Blue Flash**: Receiving data from device (RX).
  - 🟠 **Orange Flash**: Sending data to device (TX).
- **Onboard LED**: GP25 indicates an active USB connection.

## Pinout Configuration

| Function | GPIO | Physical Pin (Standard Pico) |
|----------|------|-----------------------------|
| **TX** (Output) | 0 | Pin 1 |
| **RX** (Input)  | 1 | Pin 2 |
| **GND** | - | Any Ground Pin |
| **DTR** | 2 | Pin 4 |
| **RTS** | 3 | Pin 5 |

*Note: Connect the board's **TX** to your device's **RX**, and the board's **RX** to your device's **TX**.*

## Build Instructions

### Prerequisites
- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
- ARM GCC Toolchain (`gcc-arm-none-eabi`)
- CMake

### Building
1. Create a build directory:
   ```bash
   mkdir build
   cd build
   ```
2. Configure with CMake:
   ```bash
   export PICO_SDK_PATH=/path/to/pico-sdk
   cmake ..
   ```
3. Build:
   ```bash
   make -j$(nproc)
   ```
4. Flash the resulting `uart_bridge.uf2` to your RP2040 board by holding the BOOTSEL button while connecting it.

## Project Structure
- `src/`: Source code (`main`, `usb_descriptors`).
- `include/`: Header files (`tusb_config`).
- `CMakeLists.txt`: Build configuration.
- `README.md`: This file.
