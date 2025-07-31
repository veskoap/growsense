# GrowSense Development Roadmap

This README outlines a prioritized, step-by-step list of software and hardware improvements for the GrowSense Smart Greenhouse Manager. Follow each task in sequence—complete one fully before moving on to the next.

---

## Prerequisites

- A working fork of the GrowSense codebase (Arduino sketch + supporting files)
- Completed current master branch with basic menu, sensors, fans, grow-light, buttons wired, and LCD display working
- Arduino IDE or CLI setup with all required libraries installed
- Basic familiarity with C++ (Arduino), EEPROM, and non-blocking millis() patterns

---

## ✅ Task 1. Non-Blocking Loop / Scheduler

**Goal:** Remove all `delay()` calls; replace with a millis()-based task scheduler so sensors, display, and fan control run in parallel.

1. ✅ Create a simple task scheduling approach:  
   - ✅ **SensorReadTask** (every 200 ms)  
   - ✅ **DisplayUpdateTask** (every 200 ms)  
   - ✅ **FanControlTask** (every 50 ms)
   - ✅ **PIDUpdateTask** (every 500 ms)
2. ✅ Refactor `loop()` to use millis()-based timing instead of manual delays.
3. ✅ Remove all existing `delay()` calls except for initialization.
4. ✅ Test that menu responsiveness, sensor reads, and fan ramps remain smooth.

---

## ✅ Task 2. Configuration Persistence

**Goal:** Store user settings (fan speeds, grow-light level, threshold values) in EEPROM so they survive power cycles.

1. ✅ Define a **Settings** struct:
   ```cpp
   struct Settings {
     uint8_t version;
     uint8_t coolSpeed;
     uint8_t heatSpeed;
     uint8_t growLevel;
     float   autoCoolThreshold;
     float   autoHeatThreshold;
     bool    autoMode;
     double  tempSetpoint;
     // ... other config ...
   };
   ```

2. ✅ On Select + long-press, write current Settings to EEPROM:
	- ✅ Use EEPROM.put(address, settings);
3. ✅ On startup, read back with EEPROM.get(address, settings);
4. ✅ Add a "Reset to Defaults" combo (Select + Up) to clear EEPROM and reload defaults.

---

## ✅ Task 3. PID or Fuzzy Control

**Goal:** Replace manual 0–3 speed steps with a closed-loop controller that automatically maintains a target temperature or humidity.

1. ✅ Integrate an Arduino PID library (ArduinoPID).
2. ✅ Expose menu entries for:
	- ✅ Auto Mode toggle (ON/OFF)
	- ✅ Temperature Setpoint adjustment
3. ✅ In control system:
	- ✅ Compute PID output for cooling and heating
	- ✅ Map output (0–255) to analogWrite() duty with minimum values for fan startup
	- ✅ Handle transitions between heating and cooling modes properly
4. ✅ Tune PID gains for stable response (P, I, D values).
5. ✅ Implement startup logic for both cooling and heating fans to ensure immediate response.

---

## ✅  Task 4. Menu Depth & Sub-Menus (Partially Complete)

**Goal:** Expand the menu system to include configuration sub-menus for thresholds, PID setpoints, and mode toggles.

1. ✅ Refactor menu code to support nested menus:
2. ✅ Implement Left/Right to adjust values, Up/Down to navigate, Select to enter/exit sub-menus.
3. ✅ Ensure that saving settings triggers EEPROM write.

---

## ✅ Task 5. Ultra-Fast Button System (Complete - Simplified & Optimized)

**Goal:** Implement the fastest possible button handling system for immediate user response.

1. ✅ **Ultra-Fast Button Response**:
	- ✅ Zero debounce delay for instant response
	- ✅ Single-cycle edge detection every loop iteration
	- ✅ Minimal state tracking for maximum performance
2. ✅ **Essential Functionality**:
	- ✅ Instant single button press detection
	- ✅ Long-press detection (1s) for settings save
	- ✅ Breadboard tactile button optimization
3. ✅ **Simplified Architecture**:
	- ✅ Removed complex FSM that was causing delays
	- ✅ Eliminated combo detection that interfered with basic operation
	- ✅ Direct edge detection with minimal processing overhead

**Major Performance Improvements Applied:**
- **INSTANT Response**: Eliminated all artificial delays - response limited only by main loop frequency
- **Zero-Lag Edge Detection**: Direct HIGH→LOW transition detection every loop cycle
- **Breadboard Optimized**: Designed specifically for clean 4-pin tactile buttons without bounce issues
- **Minimal State Tracking**: Ultra-lightweight FastButton struct with only essential fields
- **Speed Over Complexity**: Prioritized immediate response over advanced features
- **Hardware Trust**: Relies on quality button hardware instead of complex software debouncing
- **Single-Loop-Cycle Processing**: Button presses detected and processed within one main loop iteration

---

## ✅ Task 6. Auto Grow-Light Brightness

**Goal:** Maintain a user-set ambient-light target by automatically adjusting the RGB grow-light LEDs.  
1. ✅ **Menu entry**: under SETTINGS, add "Target Light Level" (0–5000 lx).  
2. ✅ **Sensor read**: poll current lux each cycle.  
3. ✅ **Control logic** (in `AutoLightTask`, every 200 ms):  
   - Error = target lx – current lx  
   - Compute an adjustment (e.g. simple proportional:  
     `deltaPWM = Kp * error`)  
   - Clamp and apply `analogWrite()` to R/G/B channels to raise or lower brightness.  
4. ✅ **Persist** the target level in EEPROM as part of `Settings`.  
5. ✅ **Test** in varying ambient conditions to tune the gain `Kp`.
6. ✅ **Safety features** to prevent LED burnout:
   - Maximum brightness cap
   - Gradual brightness adjustments
   - Temperature-based brightness limiting
   - Humidity-based safety cutoff
   - Duty cycling for high brightness
   - Startup safety period

---

## ✅ Task 6.5. Light Sensor Calibration (Significantly Improved - Oscillation Fixed)

**Goal:** Ensure accurate light readings and stable automatic light control.

1. ✅ **Sensor calibration**: Calibrated the TSL2591 sensor with lower gain and longer integration time
2. ✅ **Gain adjustment**: Changed to TSL2591_GAIN_LOW for better handling of brighter environments
3. ✅ **Auto-brightness stability**: Simplified PID controller with consistent parameters across all ranges
4. ✅ **Integration time**: Set to 500ms for optimal stability
5. ✅ **Filtering**: Simplified to single exponential filter to reduce lag and improve responsiveness
6. ✅ **Control system**: Unified control approach with consistent deadband and adjustment intervals
7. ✅ **Oscillation elimination**: Removed complex range-specific parameters and competing control mechanisms
8. ✅ **Simplified startup**: Removed complex startup modes that caused instability
9. ✅ **Fixed oscillation issues**: Eliminated the complex multi-layer control system causing oscillations

**Major Fixes Applied:**
- Simplified sensor filtering from complex multi-stage to single exponential filter
- Unified PID parameters (KP=0.008, KI=0.0002, KD=0.001) for all target ranges
- Fixed deadband at 15% of target with 20 lux minimum to prevent tiny oscillations
- Consistent 1-second adjustment interval for all targets for stability
- Removed complex oscillation detection system that was causing conflicts
- Eliminated range-specific control parameters that caused boundary instabilities
- Simplified brightness caps based on target ranges (20/50/100 PWM)
- Removed competing control mechanisms (successive adjustment limits, output filtering, etc.)

**Results:**
- Eliminated LED brightness oscillation issues
- More predictable and stable light control
- Faster response to target changes without overshooting
- Simplified codebase that's easier to maintain and debug

---

## ✅ Task 7. Custom Characters & Animations

**Goal:** Improve user feedback with icons and simple animations.

1. ✅ Define custom glyphs (e.g. fan, thermometer, water drop) using lcd.createChar().
2. ✅ On state changes (e.g. fan turning on), display a quick icon animation in the corner.
3. ✅ Use a non-blocking timer to manage frame updates without blocking the main loop.

---

## Task 8. Brown-Out Detection

**Goal:** Use hardware brown-out interrupt to persist critical state before undervoltage resets.

1. Enable the MCU's brown-out detection register (BOD) at ~3.8 V.
2. Attach an ISR to the BOD vector.
3. In the ISR, write current Settings and last sensor readings to EEPROM.

---

## Task 9. Wi-Fi / MQTT Integration

**Goal:** Push real-time sensor and actuator data to an MQTT broker.

1. Add Wi-Fi setup using WiFiNINA (or R4 WiFi library).
2. In NetworkTask:
	- Connect to configured SSID/pass
	- Publish JSON payloads every 5 s to topics like greensense/sensors and greensense/fans.
3. Add a Network Settings sub-menu to configure SSID, password, broker address.

---

## Task 10. Web UI

**Goal:** Host a basic web interface for remote monitoring and control.

1. Use the built-in WiFi web server to serve:
	- / → HTML dashboard showing live values
	- /set?cool=2&heat=1&grow=128 → simple control API
2. Create a minimal HTML/JS page with AJAX polling to display sensor data and send control commands.
3. Integrate auth (basic or token-based) to prevent unauthorized access.

---

## Final Notes

- Tackle each task fully before moving on.
- Keep version control commits small and focused.
- Review and test thoroughly after each change.
- Document any design decisions or deviations in the project wiki or code comments.

---

## Progress Status

- ✅ Completed tasks: 6.5
- ⚠️ Partially completed tasks: 0.5  
- ⏳ Pending tasks: 3

**Recent Major Achievement:** 
- **Ultra-Fast Button System v5.0**: Achieved zero-lag button response with single-cycle edge detection, eliminating all timing delays and providing instant user feedback optimized for breadboard tactile buttons.
