# GrowSense Smart Greenhouse Manager

A microcontroller-based greenhouse controller that reads temperature, humidity, and ambient light, drives two PWM fans (cooling & heating), a mock RGB "grow light," and provides a button-driven menu on a 16×2 I²C LCD.

---

## Table of Contents

1. [Project Overview](#project-overview)  
2. [Hardware Components](#hardware-components)  
3. [Wiring & Pinout](#wiring--pinout)  
4. [Software & Libraries](#software--libraries)  
5. [Menu & Controls](#menu--controls)  
6. [Usage](#usage)  
7. [Features](#features)
8. [Troubleshooting](#troubleshooting)  
9. [License](#license)

---

## Project Overview

GrowSense continuously monitors:

- **Temperature & Humidity** (DHT22)  
- **Ambient Light** (TSL2591)  
- **Vapor Pressure Deficit (VPD)** (calculated from temp/humidity)

Based on user-set targets (via a 5-button D-pad menu), it:

- Drives a **cooling fan** (0–3 speeds in manual mode or PID-controlled in auto mode)  
- Drives a **heating fan** (0–3 speeds in manual mode or PID-controlled in auto mode)  
- Controls an **RGB "grow light"** (0–255 brightness, or auto-adjusted to maintain target light level)  
- Maintains target temperature automatically in PID mode
- Maintains target light level automatically in auto light mode

All interaction and live readouts appear on a 16×2 I²C LCD, navigated by "Up/Down/Left/Right/Select" buttons.

---

## Hardware Components

| Component                              | Qty | Notes                                              |
|----------------------------------------|:---:|----------------------------------------------------|
| Arduino Uno R4 WiFi                    | 1   | 5 V logic; I²C on dedicated SDA/SCL pins           |
| DHT22 (AM2302) Temp & Humidity Sensor  | 1   | Data → D4 (with 4.7 kΩ pull-up)                     |
| TSL2591 Ambient Light Sensor           | 1   | I²C → SDA/SCL; polled in code                      |
| 16×2 HD44780 LCD w/ I²C Backpack (0x27) | 1   | VCC → 5 V, GND → GND, SDA/SCL → SDA/SCL             |
| 30 mm 5 V Fans                         | 2   | Cooling & Heating; low-side NPN drivers (2N2222)   |
| 2N2222 NPN Transistors                 | 5   | 2 for fans, 3 for RGB LED channels                 |
| LEDs + Resistors                       |    …| "Fan on" LEDs in parallel (330 Ω), RGB channels (330 Ω) |
| Momentary Tactile Buttons (WJW-10)     | 5   | D-pad & Select; wired to D7, D8, D11, D12, D13      |
| Common-Anode RGB LED (4-pin)           | 1   | Long pin → +5 V; each cathode via 330 Ω→2N2222      |
| Breadboard, Jumper Wires, 4.7 kΩ, 220 Ω, 330 Ω, 220 Ω, 1 kΩ resistors, flyback diodes (1N4001) |   | |

---

## Wiring & Pinout

### Power Rails  
- **+5 V rail** ← Arduino 5 V  
- **GND rail** ← Arduino GND  

### I²C Devices (share SDA/SCL)  
- LCD Backpack: VCC → 5 V, GND → GND, SDA→ SDA pin, SCL→ SCL pin  
- TSL2591: VCC→ 5 V, GND→ GND, SDA/SCL → SDA/SCL bus  

### Digital Sensors & Actuators  

| Arduino Pin | Function                                    |
|:-----------:|:--------------------------------------------|
| D2          | *unused* (formerly interrupt; now polling)  |
| D4          | DHT22 data (with 4.7 kΩ pull-up to 5 V)      |
| D7          | SELECT button (to return to main menu)      |
| D8          | LEFT button                                 |
| D11         | RIGHT button                                |
| D12         | UP button                                   |
| D13         | DOWN button                                 |
| D9          | Cooling fan driver (2N2222 base via 220 Ω)  |
| D10         | Heating fan driver (2N2222 base via 220 Ω)  |
| A0/A1/A2    | *unused*                                    |

### RGB "Grow Light" (common-anode)

- **Common anode** (longest LED pin) → +5 V rail  
- **Red cathode** → 330 Ω → 2N2222 collector; base←1 kΩ←D3; emitter→GND  
- **Green cathode** → 330 Ω → 2N2222 collector; base←1 kΩ←D5; emitter→GND  
- **Blue cathode** → 330 Ω → 2N2222 collector; base←1 kΩ←D6; emitter→GND  

### "Fan On" Indicator LEDs

- Each fan's +5 V → LED → 330 Ω → transistor collector (in parallel with fan)  

---

## Software & Libraries

- **Arduino IDE** (or CLI)  
- **Libraries**:  
  - `LiquidCrystal_PCF8574` (I²C LCD)  
  - `Adafruit_Sensor` + `Adafruit_TSL2591` (light)  
  - `DHT sensor library by Adafruit`
  - `EEPROM` (settings persistence)
  - `PID_v1` (Arduino PID library for temperature control)
- **Main Sketch**: `GrowSense.ino`  
  - Non-blocking task scheduler using millis()
  - PID temperature control for heating and cooling
  - Automatic light level control via proportional algorithm
  - EEPROM storage for settings
  - Ultra-fast button handling with zero-lag response
  - Long-press support for settings save
  - Dynamic display of sensor data & settings  

---

## Menu & Controls

1. **MAIN screen**  
   - Line 1: `T:23.4°C H:45.2%`  
   - Line 2: `L:1234 V:0.42`  
2. **COOL menu**  
   - Adjust cooling fan speed (0–3) with Left/Right; Up/Down to switch menus  
3. **HEAT menu**  
   - Adjust heating fan speed (0–3)  
4. **GROW menu**  
   - Adjust RGB grow-light brightness (0–100)
5. **AUTO mode menu**
   - Toggle automatic temperature control (ON/OFF)
6. **SETPOINT menu**
   - Adjust target temperature for automatic control
7. **TARGET LIGHT menu**
   - Adjust target light level (0-5000 lux) for auto light mode
8. **LIGHT AUTO menu**
   - Toggle automatic light level control (ON/OFF)
9. **Special Controls**
   - **SELECT**: From any sub-menu, returns to MAIN screen  
   - **Long-press SELECT**: Save settings to EEPROM (hold for 1 second)

---

## Usage

1. Wire components per the **Wiring & Pinout** section.  
2. Install required libraries via Library Manager.  
3. Upload `GrowSense.ino` to your Uno R4 WiFi.  
4. Power up—LCD shows "GrowSense Init," then MAIN screen.  
5. Navigate with D-pad and set speeds/brightness.
   - **INSTANT**: Zero-lag button response with single-cycle edge detection!
   - **ULTRA-FAST**: Lightning-fast menu navigation with 25ms display refresh rate!
6. For automatic temperature control:
   - Navigate to AUTO menu and set to ON
   - Navigate to SETPOINT menu and adjust to desired temperature
   - System will automatically control heating and cooling to maintain target temperature
7. For automatic light level control:
   - Navigate to TARGET LIGHT menu and set desired light level (lux)
   - Navigate to LIGHT AUTO menu and set to ON
   - System will automatically adjust grow light to maintain target light level
8. Long-press SELECT to save your settings for next power-up.

---

## Features

### Non-blocking Task Scheduler
- All operations run concurrently without delays
- Sensor readings, display updates, and fan control all operate independently
- Precise timing for each task: sensors (200ms), display (200ms), outputs (50ms), PID (500ms), light control (200ms)

### PID Temperature Control
- Closed-loop control for precise temperature maintenance
- Separate PID controllers for heating and cooling
- Deadband implementation to prevent oscillation (±0.2°C)
- Special startup logic for both fans to ensure immediate response
- Clean transitions between heating and cooling modes

### Enhanced Automatic Light Control
- Advanced PID (Proportional-Integral-Derivative) control algorithm maintains target light level
- Simplified, stable control system eliminates oscillation issues
- Single exponential filter for light sensor readings reduces lag and improves responsiveness
- Fixed 15% deadband (minimum 20 lux) prevents oscillation around target value
- Unified control parameters work consistently across all light target ranges
- Comprehensive safety features:
  - Target-specific maximum brightness caps (20/50/100 PWM based on target)
  - Temperature-based brightness limiting (reduces as temperature rises)
  - Humidity-based safety cutoff (disables at >95% humidity)
  - Duty cycling for high brightness (70% on, 30% reduced)
  - Conservative startup behavior with safe initial brightness
  - Minimum on-time enforcement to prevent rapid cycling
- Calibration mode for tuning and testing (2+ second long-press on Select)

**Recent Improvements:**
- **Eliminated LED brightness oscillation issues** through simplified control system
- Removed complex multi-stage filtering that was causing control lag
- Unified PID parameters across all target ranges for consistent behavior  
- Fixed adjustment intervals at 1 second for all targets to improve stability
- Simplified startup behavior eliminates complex state machines

### Configuration Persistence
- All settings stored in EEPROM
- Settings automatically loaded at startup
- Manual save with long-press of SELECT button
- Factory reset option with SELECT+UP button combo

### Ultra-Fast, Zero-Lag Button Interface v5.0 - MAXIMUM SPEED
- **INSTANT Response**: Zero debounce delay - immediate button press detection  
- **Single-Cycle Edge Detection**: Direct edge detection every loop cycle for maximum speed
- **Minimal State Tracking**: Ultra-lightweight processing optimized for breadboard tactile buttons
- **No Artificial Delays**: Response limited only by main loop frequency (microseconds)
- **Breadboard Optimized**: Designed specifically for clean 4-pin tactile buttons
- **INSTANT menu navigation**: Lightning-fast display updates with 25ms refresh rate
- **Zero Timing Windows**: No complex FSM or debounce logic to cause delays
- **Essential Functionality**: Single press detection and long press (1s) for settings save
- **Maximum Performance**: Speed prioritized over advanced features for immediate response
- **Single-Loop-Cycle Detection**: Button presses detected and processed within one main loop iteration
- **Hardware Trust**: Relies on quality button hardware instead of software complexity
- **Pure Speed Focus**: Eliminates all unnecessary processing for instant user feedback

### Real-time Sensor Monitoring
- Temperature and humidity readings from DHT22
- Light level from TSL2591
- Calculated Vapor Pressure Deficit (VPD)
- Continuous updates in main display

---

## Troubleshooting

- **UP button not working** → FIXED: Simplified button handling eliminates complex FSM bugs.
- **Menu navigation slow or unresponsive** → COMPLETELY SOLVED with Ultra-Fast v5.0: Zero debounce delay, single-cycle edge detection, and 25ms display refresh provide instant response.
- **Buttons not responsive or missing presses** → COMPLETELY SOLVED with direct edge detection that captures every button press instantly.
- **Button presses taking 1.5+ seconds to register** → COMPLETELY SOLVED: Eliminated all debounce delays and timing windows for immediate response.
- **Rapid button clicks not registering** → COMPLETELY SOLVED: Single-loop-cycle detection processes every button press regardless of speed.
- **Settings messages blocking navigation** → Non-blocking temporary messages (1.5s) allow continuous navigation without delays.
- **Inconsistent button response** → COMPLETELY SOLVED with minimal state tracking providing consistent, immediate response every time.
- **Button handling complexity causing issues** → SIMPLIFIED: Removed FSM, combos, and repeat logic for maximum reliability and speed.
- **LED turns on at startup** → Startup safety period added; LED remains off for first 5 seconds.
- **LED brightness oscillating** → Fixed with simplified PID control system; oscillation issues eliminated.
- **LED too bright in auto mode** → Brightness caps and duty cycling implemented; target-specific maximum values prevent overbrightness.
- **Fan won't spin at low speeds** → minimum PWM raised to ensure start torque (180 for heating, 150 for cooling).
- **Fan oscillating between modes** → PID controllers now properly reset during mode transitions.
- **Heating fan slow to start** → Improved startup logic with higher initial PWM.
- **Light reading stuck at 0** → switched to polling `getFullLuminosity()` each loop.  
- **LCD garbled** → verify I²C address (0x27 vs 0x3F) and pull-ups on SDA/SCL.
- **Settings not saving** → use long-press on SELECT button to save settings.

---

## License

Distributed under the MIT License. See `LICENSE` for details.  