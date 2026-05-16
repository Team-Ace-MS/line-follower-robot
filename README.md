# 🏎️ Line Follower Robot (LFR)

Welcome to the primary development hub for our high-speed autonomous Line Follower Robot. This repository centralizes all source code, printed circuit board (PCB) layouts, firmware logic architectures, and mechanical chassis designs.

---

## 🛠️ System Specifications & Tech Stack
* **Core Controller:** Raspberry Pi Pico 2 (RP2350 Architecture)
* **Motor Driver:** Dual Channel TB6612FNG
* **Voltage Regulator:** LM2596 Buck Converter
* **Sensor Array:** QRE1113 16-Sensor Matrix with Multiplexed Reading Logic
* **UI for User Control:** Adafruit SSD1306 I2C 128X64 OLED display
* **Motors:** 800 RPM GA12-N20 Gear Motors
* **Hardware Framework:** PCB, 3D Printed Chassis, Fiberglass Veroboard
* **Firmware Framework:** C++ / Arduino Engine
* **Control Frequency:** 1000Hz Hardware Timer Interrupt
* **Hardware Design EDA:** KiCad, EasyEDA

---

## 📂 Repository Directory Guide
* `/firmware` — Core execution code, PID tuning parameters, OLED display code, and calibration profiles.
* `/hardware` — Schematic diagrams (PDF format) and production-ready PCB layouts.
* `/docs` — System flowcharts, component datasheets, and algorithm math models.
* `/mechanical` — 3D printing STL models and primary structural chassis dimensions.

---

## 📈 High-Level Execution Flow
This logic loop is hardcoded to run via interrupts to ensure absolute predictability at high velocities.

```mermaid
graph TD
    Start([Power On]) --> Init[Initialize Peripherals & Callibrate Sensors]
    Init --> Read[Mux Array: Scan 16 Sensors]
    Read --> Calc[Compute Position Error Vector]
    Calc --> PID[Execute 1000Hz PID Algorithim]
    PID --> Drive[Adjust Left/Right Motor PWM Output]
    PID --> Cross[non-PID Path Logic Override]
    Cross --> Drive
    Drive --> Update[OLED Display and Time Counters]
    Update --> Read
    Drive --> End[End When Solid Black Box]
