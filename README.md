# Amon Link

**Project Status:** In Progress  

**Short description:** USB-powered RF bridge between **Amon Ground Control** (PC) and the **Amon Lander** drone, built around the WCH **CH32V003F4P6** MCU with **dual NRF24L01 + RFX2401C** radios for robust range. 

> Part of the Amon Lander project:
>
> - **Amon Lander** — overall vehicle, frame, test/control SW, and data  
>   https://github.com/TilenTinta/Amon_Lander
> - **Amon Board** — flight controller hardware & firmware  
>   https://github.com/TilenTinta/Amon_Board
> - **Amon Ground Control** — desktop GCS for telemetry & control  
>   https://github.com/TilenTinta/Amon_GroundControl_Software

![Amon Link](https://github.com/TilenTinta/Amon_Link/blob/main/Images/Assembly%20v5.png)  

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
  - [Hardware](#hardware)
  - [Software & Compatibility](#software--compatibility)
  - [Diagnostics & Controls](#diagnostics--controls)
- [Quick Start](#quick-start)
- [Flashing Firmware](#flashing-firmware)
- [Debugging](#debugging)
  - [LED Status Codes](#led-status-codes)
  - [Button Usage](#button-usage)
- [Repository Structure](#repository-structure)
- [Roadmap](#roadmap)
- [Status & Disclaimer](#status--disclaimer)
- [License](#license)
- [Acknowledgments](#acknowledgments)

---

## Overview

**Amon Link** is a compact, low-power **RF communication bridge** that connects a PC-based Ground Control application to the Amon Lander over 2.4 GHz radios. It prioritizes reliability by pairing **two NRF24L01 modules** with the **RFX2401C PA/LNA** front-end for extended range and improved link quality. The board is powered from **USB Type‑C** and integrates a **3.3 V LDO**. 

---

## Features

### Hardware

- **MCU:** WCH **CH32V003F4P6**. 
- **Power:** USB‑powered with onboard **3.3 V LDO**. 
- **Connector:** **USB Type‑C**. 
- **RF Link:** Dual **NRF24L01** radios with **RFX2401C (PA+LNA)** for long range and better SNR. 
- **Indicators/Controls:** Status LED and **reconnect button**. 

### Software & Compatibility

- **Firmware:** C, developed in **MounRiver Studio IDE**. 
- **USB–UART:** **CH340G** bridge (install drivers if needed). 
- **Platforms:** Works with **Windows, Linux, and macOS** Ground Control apps. 
- **Updates:** **Custom bootloader** supports USB firmware updates. 

### Diagnostics & Controls

- **Status LED** for connection state. 
- **Button** to trigger manual reconnect. 

---

## Quick Start

1. **Connect** the Amon Link board to your PC using **USB‑C**.   
2. **Install drivers** if required for **CH340G**.   
3. **Place antennas** for the two NRF24L01 radios to ensure clear line‑of‑sight where possible.   
4. **Launch** the Amon Ground Control application and select the appropriate COM port.

---

## Flashing Firmware

- Open the project in **MounRiver Studio IDE**, compile, and **upload** to the **CH32V003F4P6** MCU. 

> The board also includes a **custom bootloader** for **USB firmware updates** which is supported by the desktop software. 

### Bootloader
Firmware for device also includes bootloader option. To flash that:
- First flash main firmware (folder: **Firmware**),
- after that flash bootloader (folder: **Bootloader**)

If bootloader is not used you **MUST** modify the code in folder **Firmware**. In file `Link.ld` change code on the top of the file to:

```c
/* Full flash 16KB */
__app_size = 16K;

MEMORY
{
  /* Application region - 16KB */  
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = __app_size 

  /* RAM - untouched 2KB */  
	RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 2K
}

SECTIONS
{
  .init :
  {
    ...
```
Once the bootloader is flashed, you can put your device in firmware update mode by holding the **Reconnect** button while plugging in the USB cable or by triggering it via software. When the blue and red LEDs are lights up, you're in firmware update mode!

The firmware can be updated using a graphical application. The application is built with a Python backend and an Electron front end. You can select and import either a hex or a binary file. Once imported, the CRC32 checksum is calculated for the file.

This checksum is used during device connection. When the device connects, a handshake process exchanges several pieces of data, one of which is the CRC32 value of the firmware currently installed on the device. If this CRC matches the one from the new file, you are notified before proceeding with the update. After a successful update, the new CRC32 value is written to the device for future reference.

![Amon Link fw app](https://github.com/TilenTinta/Amon_Link/blob/main/Images/FW_Update_app.png)  

---

## Debugging

### LED Status Codes  

| LED Pattern (App)     | Meaning                  |
|-----------------|---------------------------|
| Constant blinking **red**   | Device initialization failed |
| Solid **blue**    | Device initialized           |
| Blinking **red**     | Received packet          |

| LED Pattern (Boot)     | Meaning                   |
|-----------------|---------------------------|
| Solid **blue** and **red**    | Waiting on USB data    |


### Button Usage  

Press the **button** to trigger a manual **reconnect** attempt or befor powring up device to enter bootloader mode.


## Range testing
The plan for this device is not to reach over 1 km of range, but only a few meters are enough for now. To test this, I did a simple test. I connected the device to my laptop and established a connection from the **Ground control app** to **Amon Link** and from that to **Amon Lander**. After receiving telemetry data, I started moving away from the drone. This test was done with line of sight between the drone and link device.

![Range test](https://github.com/TilenTinta/Amon_Link/blob/main/Images/PXL_20260131_140538853.jpg)  

Because of field limitations, the test was not done until telemetry cut-off. The test will be repeated in the future to really get the limitation of radio range.

![Range test](https://github.com/TilenTinta/Amon_Link/blob/main/Images/Distance_test.png)  

---

## Status & Disclaimer

**In active development.** The hardware and firmware are usable, but changes are ongoing; expect rough edges as features evolve. 
