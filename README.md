# Smart Traffic Light Controller (AVR, C)

## Overview
A professional, efficient, and extensible smart traffic light controller for AVR microcontrollers (e.g., ATmega328P). This project demonstrates real-time, sensor-driven intersection management with emergency override, pedestrian crossing, and advanced embedded techniques. Designed for simulation in SimulIDE and easy adaptation to real hardware or PCB design (KiCad).

---

## Features
- **Sensor-based Timing:** Dynamically adjusts green light duration based on vehicle presence.
- **Pedestrian Button:** Safe, latched pedestrian crossing with blinking warning.
- **Emergency Override:** Immediate priority for emergency vehicles.
- **Debounced Inputs:** Reliable button and sensor handling.
- **Non-blocking State Machine:** Uses timer interrupts for responsive, real-time operation.
- **UART Debug Output:** Serial messages for state transitions and debugging.
- **Extensible Design:** Easy to add more features or directions.

---

## Hardware Requirements
- **Microcontroller:** ATmega328P (or compatible AVR)
- **Simulation:** [SimulIDE](https://www.simulide.com/p/downloads.html) (free, open-source)
- **Optional:** KiCad for PCB design
- **Inputs:**
  - 2x Vehicle sensors (digital, active low)
  - 2x Pedestrian push buttons (digital, active low)
  - 1x Emergency override button (digital, active low)
- **Outputs:**
  - 6x Traffic light LEDs (Red, Yellow, Green for NS and EW)
  - 2x Pedestrian LEDs (NS, EW)

---

## Pin Mapping (ATmega328P Example)
| Function              | Pin (AVR) |
|-----------------------|-----------|
| NS Vehicle Sensor     | PD2       |
| EW Vehicle Sensor     | PD3       |
| NS Pedestrian Button  | PD4       |
| EW Pedestrian Button  | PD5       |
| Emergency Override    | PD6       |
| NS Red                | PB0       |
| NS Yellow             | PB1       |
| NS Green              | PB2       |
| EW Red                | PB3       |
| EW Yellow             | PB4       |
| EW Green              | PB5       |
| NS Pedestrian Light   | PB6       |
| EW Pedestrian Light   | PB7       |

---

## How It Works
- **Default:** North-South (NS) green, East-West (EW) red.
- **Vehicle Detected:** Switches to EW green after NS phase if EW vehicle present.
- **Pedestrian Button:** Latches request; crossing is granted after current green phase, with blinking warning in last second.
- **Emergency Override:** All red except NS green; resumes normal operation when cleared.
- **Non-blocking:** All timing is handled by a timer interrupt, so the system is always responsive.
- **Debugging:** State changes are output via UART (9600 baud).

---

## Build Instructions
1. **Requirements:**
   - AVR-GCC toolchain (e.g., [WinAVR](https://sourceforge.net/projects/winavr/))
   - SimulIDE for simulation
2. **Build:**
   ```sh
   make
   # or manually:
   avr-gcc -mmcu=atmega328p -DF_CPU=16000000UL -Os -o traffic_controller.elf traffic_controller.c
   avr-objcopy -O ihex traffic_controller.elf traffic_controller.hex
   ```
3. **Simulate:**
   - Open SimulIDE
   - Place ATmega328P, connect LEDs and buttons as per pin mapping
   - Load `traffic_controller.hex` into the MCU
   - (Optional) Connect serial monitor to view debug output (9600 baud)

---

## Advanced Implementation Details
- **Debouncing:** All digital inputs are debounced in software for reliability.
- **State Machine:** Each traffic phase is a state; transitions are time- and event-driven.
- **Timer Interrupt:** Timer1 generates a 1ms tick for all timing, enabling non-blocking operation.
- **Pedestrian Blinking:** Pedestrian light blinks during the last second of crossing for safety.
- **UART Debug:** State transitions and emergency events are sent over UART for easy monitoring.

---

## Customization
- **Timing:** Adjust `GREEN_MIN`, `YELLOW_TIME`, `PED_TIME`, and blink intervals in `traffic_controller.c`.
- **Pin Mapping:** Change pin assignments as needed for your hardware.
- **Extending:** Add more states, directions, or features by expanding the state machine.

---
