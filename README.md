# 🚨 Dual-Mode Rescue Communicator
### STM32 · BLE · ZigBee · GPS

> **Real-time victim detection and alert transmission system for flood disaster response** — combining Bluetooth Low Energy short-range scanning with ZigBee long-range mesh networking on an STM32 microcontroller.

---

![Platform](https://img.shields.io/badge/Platform-STM32F4-blue?style=flat-square&logo=stmicroelectronics)
![Language](https://img.shields.io/badge/Language-C-A8B9CC?style=flat-square&logo=c)
![Protocol](https://img.shields.io/badge/Wireless-BLE%20%7C%20ZigBee-green?style=flat-square)
![GPS](https://img.shields.io/badge/Location-GPS%20NMEA-orange?style=flat-square)

---

## 📌 Table of Contents

- [Overview](#overview)
- [Problem Statement](#problem-statement)
- [System Architecture](#system-architecture)
- [Key Features](#key-features)
- [Hardware Components](#hardware-components)
- [How It Works](#how-it-works)
- [Core Embedded Concepts](#core-embedded-concepts)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
- [Results](#results)
- [Future Scope](#future-scope)
- [Team](#team)

---

## Overview

The **Dual-Mode Rescue Communicator** is an embedded system designed to assist rescue teams in locating stranded flood victims efficiently. It uses a **patrol-first detection approach** — the rescue unit autonomously scans disaster zones, detects human BLE beacons, acquires GPS coordinates, and relays priority alerts to a remote coordination center over ZigBee.

---

## Problem Statement

Natural disasters — particularly floods — devastate communication infrastructure right when it's needed most. Rescue teams face:

- No real-time location data for stranded victims
- Manual search operations that are slow, risky, and error-prone
- Cellular network failure in disaster-affected regions
- Inability to manage multiple simultaneous rescue missions

This project directly addresses each of these gaps with a low-power, infrastructure-independent embedded solution.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────┐
│                    MOBILE RESCUE UNIT                    │
│                                                          │
│  ┌─────────┐   UART   ┌──────────┐   UART   ┌────────┐  │
│  │  BLE    │ ────────▶│  STM32   │◀──────── │  GPS   │  │
│  │ Scanner │          │ (Core MCU)│          │ Module │  │
│  └─────────┘          └────┬─────┘          └────────┘  │
│                            │ UART                        │
│                      ┌─────▼──────┐                     │
│                      │  ZigBee TX │                      │
│                      └─────┬──────┘                      │
└────────────────────────────┼────────────────────────────┘
                             │ ZigBee Mesh
                   ┌─────────▼──────────┐
                   │  RESCUE COORDINATION│
                   │       CENTER        │
                   │  (ZigBee RX + PC)  │
                   └────────────────────┘
```

**Flow:**
1. BLE scanner detects a human beacon (smartphone / emergency app)
2. STM32 triggers GPS acquisition via UART interrupt
3. NMEA sentence parsed → lat/long extracted
4. Coordinates packaged as IMP alert, sent over ZigBee
5. Coordination center receives alert, marks location on map
6. Patrol continues — concurrent missions supported without interruption

---

## Key Features

| Feature | Description |
|---|---|
| 🔵 BLE Human Detection | Scans for BLE advertisement packets from smartphones/beacons |
| 📡 ZigBee Mesh Alerting | Long-range, low-power alert transmission (no cellular needed) |
| 🛰️ GPS Coordinate Parsing | Real-time NMEA sentence parsing for precise victim location |
| ⚡ Non-blocking Architecture | Interrupt-driven design — no polling loops blocking operation |
| 🔁 Multi-mission Support | Marks IMP zones while continuing active patrol |
| 🔋 Low Power Design | BLE + ZigBee chosen for energy efficiency in field deployment |

---

## Hardware Components

| Component | Role |
|---|---|
| **STM32F4xx** | Core microcontroller — UART, timers, GPIO, interrupts |
| **HM-10 / CC2541** | BLE 4.0 module — human presence detection |
| **XBee / CC2530** | ZigBee module — long-range mesh alert transmission |
| **NEO-6M GPS** | GPS module — NMEA data, coordinate acquisition |
| **LEDs / Buzzer** | Visual/audio feedback for detection events |
| **Power Supply** | 3.3V / 5V regulated supply for field deployment |

---

## How It Works

### Phase 1 — BLE Scan (Detect)
The STM32 sends AT commands to the BLE module over UART. The module continuously listens for BLE advertisement packets. When a human-associated beacon is detected (RSSI threshold crossing), an interrupt is raised.

### Phase 2 — GPS Acquisition (Locate)
Upon BLE detection, the STM32 activates the GPS UART channel. Incoming NMEA sentences (`$GPRMC`, `$GPGGA`) are parsed character-by-character using interrupt-driven UART receive. Latitude and longitude are extracted and formatted.

### Phase 3 — ZigBee Alert (Communicate)
The GPS coordinates are packaged into a structured alert string:
```
IMP|LAT:17.3850|LON:78.4867|STATUS:VICTIM_DETECTED
```
This is transmitted over UART to the ZigBee module, which relays it through the mesh network to the coordination center.

### Phase 4 — Continued Patrol
The system flags the location as an IMP zone, resets state, and resumes BLE scanning — enabling simultaneous multi-victim handling without human intervention.

---

## Core Embedded Concepts

This project demonstrates practical implementation of:

- **UART Communication** — Multi-channel UART for BLE, GPS, and ZigBee modules
- **Interrupt-Driven Programming** — USART RX interrupts, EXTI lines for event handling
- **Timer & PWM Configuration** — TIM peripherals for non-blocking delays and PWM signaling
- **Real-Time Event Handling** — Priority-based ISR management, state machine design
- **GPS NMEA Parsing** — Tokenization and extraction of `$GPRMC` / `$GPGGA` sentences
- **Peripheral Interfacing** — GPIO, USART, TIM on STM32 HAL / bare-metal

---

## Project Structure

```
DUAL-MODE-RESCUE-COMMUNICATOR-USING-STM32-BLE-ZIGBEE/
│
├── project_code/            # STM32 firmware source
│   ├── Core/
│   │   ├── Src/
│   │   │   ├── main.c           # Application entry point
│   │   │   ├── ble_handler.c    # BLE scan and detection logic
│   │   │   ├── gps_parser.c     # NMEA sentence parser
│   │   │   ├── zigbee_comm.c    # ZigBee alert transmission
│   │   │   └── uart_driver.c    # UART abstraction layer
│   │   └── Inc/
│   │       └── *.h
│   └── Makefile
│
├── documentation/           # Project report, block diagrams, schematics
│
├── Project_Hardware_Images/ # Photos of assembled hardware
│
└── README.md
```

---

## Getting Started

### Prerequisites

- STM32CubeIDE or STM32CubeMX + ARM-GCC toolchain
- ST-Link V2 programmer/debugger
- Serial terminal (TeraTerm / PuTTY / minicom) for ZigBee output monitoring

### Build & Flash

```bash
# Clone the repository
git clone https://github.com/Raghu0033/DUAL-MODE-RESCUE-COMMUNICATOR-USING-STM32-BLE-ZIGBEE.git
cd DUAL-MODE-RESCUE-COMMUNICATOR-USING-STM32-BLE-ZIGBEE/project_code

# Build using Makefile
make all

# Flash to STM32 via ST-Link
make flash
```

### Serial Monitor Setup

Connect the ZigBee coordinator to a PC and open a serial terminal:
- **Baud Rate:** 9600
- **Data Bits:** 8
- **Stop Bits:** 1
- **Parity:** None

You will see incoming alert packets like:
```
[ALERT] IMP ZONE DETECTED
LAT: 17.385012  LON: 78.486671
STATUS: VICTIM_CONFIRMED
```

---

## Results

| Metric | Result |
|---|---|
| BLE Detection Range | ~10 m (indoor), ~20 m (open field) |
| GPS Fix Time | < 30 seconds (cold start), < 5 sec (warm) |
| ZigBee Alert Latency | < 500 ms end-to-end |
| ZigBee Range | ~100 m (line-of-sight) |
| Multi-mission Handling | ✅ Concurrent IMP zone tracking verified |
| Power Consumption | Low — suitable for battery-powered field unit |

---

## Future Scope

- [ ] LoRa integration for extended range (5–10 km) in open terrain
- [ ] OLED display module for on-device status feedback
- [ ] Android companion app for coordination center visualization
- [ ] Machine learning–based BLE signal filtering to reduce false positives
- [ ] Drone-mounted variant for aerial patrol and victim mapping
- [ ] Solar charging support for extended field deployment

---

## Team

| Name |
|----|
|Veera Raghavendra |
|Mohan Sai santhosh|
|Rohit Varma|

---
