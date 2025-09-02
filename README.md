
# EEE 416 – Experiment 02 (STM32F446RE, CMSIS/Keil)

Completed project for **BUET EEE 416: Microprocessor & Embedded Systems Lab – Experiment 02**.

**Features**
- LED on **PA5**
- Pushbutton on **PC13** (active‑LOW, pull‑up)
- **28BYJ‑48** stepper motor via **ULN2003** driver on **PC5, PC6, PC8, PC9**
- Behavior:
  - Button **released** → LED **OFF**, **full‑step CCW**
  - Button **pressed** → LED **blinks**, **half‑step CW**

## Hardware
- Nucleo‑F446RE (or equivalent STM32F446RE board)
- ULN2003 + 28BYJ‑48 stepper
- 1 × LED + 220–1kΩ series resistor
- Pushbutton (PC13 to GND), internal pull‑up enabled

## Pin Map
| Function | Port/Pin |
|---------:|:---------|
| LED      | PA5      |
| Button   | PC13     |
| Step A   | PC5      |
| Step B   | PC6      |
| Step C   | PC8      |
| Step D   | PC9      |

## Build (Keil MDK-ARM)
Open: `STM32F4_Keil/EEE416-Exp02_Stepper/DSMCprojecttemplate1.uvprojx`  
Compile and download to the board. No HAL required—pure CMSIS + startup files are included.

## Tuning
If the motor stalls, increase the in‑code delays:
```c
const uint32_t HALF_DELAY_MS = 5;
const uint32_t FULL_DELAY_MS = 5;
```



> The Keil project was derived from the provided lab template and adapted to complete Experiment 02.
