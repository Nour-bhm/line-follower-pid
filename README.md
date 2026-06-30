# line follower pid
An autonomous, line-following robot prototype built for the Polymaze competition. This repository contains the complete open-source hardware, electronics, and firmware design.

## Repository Structure
* `/Firmware` - Arduino C++ source code utilizing a custom PID control loop.
* `/CAD` - 3D printable chassis components (.STL formats).
* `/PCB` - Circuit schematics.

---
## System Architecture

### 1. Firmware & Control Logic
The robot relies on a reactive embedded system to process sensor data in real time:
* **Microcontroller:** Arduino Nano
* **Control Loop:** A tuned Proportional-Integral-Derivative (PID) algorithm that dynamically adjusts differential steering based on error margins from the sensor array.

### 2. Electrical & PCB Design
Custom power distribution circuit designed to isolate high-current inductive motor noise from the logic unit:
* **Motor Driver:** TC1508A
* **Sensors:** 8-Channel QTR Infrared sensor array for precise line tracking
* **Power Management:** Step-down buck converters regulating 7.4V LiPo input to stable 5V logic rails.

### 3. Mechanical & CAD Design
A lightweight, high-traction differential-drive chassis engineered for rapid assembly and low center of mass:
* **Actuators:** Standard 1:48 dual-shaft yellow DC motors (slated for N20 micro-motor upgrades in V2).
* **Chassis:** Custom 3D-printed modular baseplates optimized for component placement and structural rigidity under high-speed impacts.

---

## Future Technical Roadmap
1. **Hardware:** Transition to high-precision N20 micro metal gearmotors for tighter angular velocity control & an TB6612FNG motor driver for pwm control.
3. **Software:** Implement a predictive maze-solving algorithm (e.g., Flood Fill) early in the simulation phase.

---
