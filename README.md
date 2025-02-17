# Amon_Link
Project Status:
PROJECT IN PROGRESS...

Amon_Link is an open-source electronic module designed for RF communication between a PC-based Ground Control system and the Amon lander drone. It serves as a reliable and efficient communication bridge, utilizing dual radios and a compact, low-power design.

![Amon Link](https://github.com/TilenTinta/Amon_Link/blob/main/Images/Assembly%20v5.png)
---

# Features
## Hardware
- Microcontroller: WCH CH32V003F4P6
- Power Supply: USB-powered with onboard LDO regulator (3.3V)
- Connectivity: USB Type-C
- RF Communication:
    - Dual NRF24L01 radios with RFX2401C (LNA + PA) for extended range and improved signal quality

## Diagnostics & Controls:
- Status LED indicator
- Button for manual reconnecting

## Software & Compatibility
- Firmware Development: C in MounRiver Studio IDE
- Supported Platforms: Compatible with Windows, Linux, and macOS for Ground Control applications
- Driver Requirements: CH340G IC for USB communication requires appropriate drivers (if needed)
---

# Usage
## Setting Up

Hardware Setup:
- Connect the board via USB-C to the PC.
- Ensure proper antenna placement for optimal RF performance.

## Flashing Firmware:
- Use MounRiver Studio IDE to compile and upload the firmware.

## Debugging
LED Status Indications:
- Blinking red: device initialization
- Blinking blue: connecting...
- Solid blue: Connection established
- Solid red: Connection failed

## Button Usage:
Press to trigger reconnecting 