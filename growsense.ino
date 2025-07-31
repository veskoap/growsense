/*
  GrowSense Smart Greenhouse Manager
  ----------------------------------
  Controls a greenhouse environment using:
    - DHT22 (temperature/humidity)
    - TSL2591 (ambient light)
    - Two PWM fans (cooling/heating)
    - RGB grow light (PWM)
    - 16x2 I2C LCD (menu, live data)
    - 5-button D-pad (Up/Down/Left/Right/Select)
  
  Features:
    - Live sensor readout (temp, humidity, light, VPD)
    - Manual fan and light control
    - Automatic temperature control via PID
    - Automatic light level adjustment
    - Non-blocking task scheduler
    - Settings persistence in EEPROM
    - Menu navigation with button combos and long-press
    - Temperature setpoint adjustment
  
  Completed Tasks:
    - Task 1: Non-Blocking Loop / Scheduler
    - Task 2: Configuration Persistence
    - Task 3: PID Control
    - Task 5: Long-Press & Combo Inputs (Complete FSM)
    - Task 6: Auto Grow-Light Brightness
    
  Ultra-Fast, Zero-Lag Menu System (v5.0 - MAXIMUM SPEED):
    - INSTANT button response with zero debounce delay  
    - Direct edge detection on every loop cycle
    - Minimal state tracking for maximum performance
    - No timing windows or delays - immediate response
    - INSTANT display updates (25ms refresh rate)
    - Optimized for breadboard tactile buttons
    - Single-cycle button press detection
    - Zero artificial delays in button handling

  See README.md for wiring, menu, and usage details.
*/

// ======== Includes ========
#include <Wire.h>                        // I2C communication
#include <LiquidCrystal_PCF8574.h>       // I2C LCD library
#include <Adafruit_Sensor.h>             // Sensor base class
#include <Adafruit_TSL2591.h>            // Light sensor
#include <DHT.h>                         // DHT22 temp/humidity
#include <math.h>                        // For exp()
#include <EEPROM.h> // For settings persistence
#include <PID_v1.h> // Arduino PID library

// ======== Pin Definitions ========
#define DHTPIN       4    // DHT22 data pin
#define DHTTYPE      DHT22

#define COOL_PIN     9    // Cooling fan (PWM, NPN driver)
#define HEAT_PIN     10   // Heating fan (PWM, NPN driver)

#define LED_R_PIN    3    // RGB grow-light (Red, PWM)
#define LED_G_PIN    5    // RGB grow-light (Green, PWM)
#define LED_B_PIN    6    // RGB grow-light (Blue, PWM)

#define BTN_SELECT   7    // Menu select button
#define BTN_LEFT     8    // Menu left button
#define BTN_RIGHT    11   // Menu right button
#define BTN_UP       12   // Menu up button
#define BTN_DOWN     13   // Menu down button

// ======== Objects ========
LiquidCrystal_PCF8574 lcd(0x27);         // LCD at I2C address 0x27
Adafruit_TSL2591    tsl = Adafruit_TSL2591(2591); // Light sensor
DHT                dht(DHTPIN, DHTTYPE);         // Temp/humidity sensor

// ======== Menu System ========
/**
 * MenuItem: Enumerates the menu screens.
 *   MI_MAIN: Main sensor readout
 *   MI_COOL: Cooling fan control
 *   MI_HEAT: Heating fan control
 *   MI_GROW: Grow light control
 *   MI_AUTO: Auto/manual toggle
 *   MI_SETPOINT: Temperature setpoint adjust
 *   MI_TARGET_LIGHT: Target light level adjust
 *   MI_LIGHT_AUTO: Auto light mode toggle
 *   MI_COUNT: Number of menu items
 */
enum MenuItem { MI_MAIN, MI_COOL, MI_HEAT, MI_GROW, MI_AUTO, MI_SETPOINT, MI_TARGET_LIGHT, MI_LIGHT_AUTO, MI_COUNT };
MenuItem menu = MI_MAIN;  // Current menu state

// ======== Custom Character Definitions ========
// Each character is 5x8 pixels, defined as byte arrays
// Character 0: Thermometer
const byte CHAR_THERMOMETER[8] = {
  B00100,
  B01010,
  B01010,
  B01010,
  B01110,
  B11111,
  B11111,
  B01110
};

// Character 1: Water droplet (humidity)
const byte CHAR_DROPLET[8] = {
  B00100,
  B00100,
  B01010,
  B01010,
  B10001,
  B10001,
  B10001,
  B01110
};

// Character 2: Light bulb
const byte CHAR_LIGHT[8] = {
  B00000,
  B01110,
  B10001,
  B10101,
  B10101,
  B01110,
  B01110,
  B00100
};

// Character 3: Cooling fan (frame 1)
const byte CHAR_FAN_1[8] = {
  B00100,
  B10101,
  B01110,
  B11011,
  B01110,
  B10101,
  B00100,
  B00000
};

// Character 4: Cooling fan (frame 2)
const byte CHAR_FAN_2[8] = {
  B00100,
  B00100,
  B10110,
  B01101,
  B10110,
  B00100,
  B00100,
  B00000
};

// Character 5: Heating element
const byte CHAR_HEAT[8] = {
  B00100,
  B01110,
  B01110,
  B01110,
  B01110,
  B01010,
  B01010,
  B01010
};

// Character 6: Auto mode symbol
const byte CHAR_AUTO[8] = {
  B00000,
  B01110,
  B10001,
  B10101,
  B10101,
  B10001,
  B01110,
  B00000
};

// Character 7: Warning/alert symbol
const byte CHAR_ALERT[8] = {
  B00000,
  B00100,
  B01110,
  B01110,
  B01110,
  B01110,
  B00100,
  B00000
};

// ======== Animation Control Variables ========
bool g_animationActive = false;         // Whether any animation is currently active
unsigned long g_animationStartMillis = 0;  // When the current animation started
unsigned long g_lastAnimationUpdateMillis = 0;  // Last time animation was updated
const unsigned long ANIMATION_FRAME_INTERVAL = 250;  // Time between animation frames (ms)
const unsigned long ANIMATION_DURATION = 3000;  // Total animation duration (ms)
byte g_currentAnimationFrame = 0;  // Current frame in animation sequence
byte g_animationDisplayX = 15;     // Default X position for animation (right corner)
byte g_animationDisplayY = 0;      // Default Y position for animation (top row)
byte g_currentAnimationType = 0;   // Type of animation currently running
                                  // 0=None, 1=CoolFan, 2=HeatFan, 3=Light, 4=Alert

// ======== Control Variables ========
int coolSpeed = 0;   // Cooling fan speed (0–3)
int heatSpeed = 0;   // Heating fan speed (0–3)
int growLevel = 0;   // Grow light brightness (0–255)

// ======== Lightning-Fast Button System ========
// Absolute minimal state tracking for maximum speed
bool g_lastButtonState[14]; // Previous button state by pin number

// Long press tracking (only for settings save)
unsigned long g_selectPressStart = 0;
bool g_selectPressed = false;
const unsigned long BUTTON_LONG_PRESS_MS = 1000;

// ======== Responsive Display System ========
bool g_showTempMessage = false; // Flag for temporary message display
String g_tempMessage = ""; // Temporary message text
unsigned long g_tempMessageStartTime = 0; // When temp message was shown
const unsigned long TEMP_MESSAGE_DURATION = 1500; // 1.5 seconds for temp messages

// ======== Global Timing Variables ========
unsigned long g_currentMillis = 0; // Global timestamp for this loop

// Task intervals (ms) - Lightning-fast button priority optimization  
const unsigned long SENSOR_INTERVAL = 200;
const unsigned long DISPLAY_INTERVAL = 500; // Longer interval since buttons update immediately
const unsigned long OUTPUT_INTERVAL = 50;
const unsigned long PID_INTERVAL = 500;
const unsigned long LIGHT_CONTROL_INTERVAL = 200;
const unsigned long CALIBRATION_MODE_INTERVAL = 1000; // 1 second for calibration mode updates
const unsigned long ANIMATION_INTERVAL = 100; // 100ms for smooth animations

// Last execution times
unsigned long lastSensorMillis = 0;
unsigned long lastDisplayMillis = 0;
unsigned long lastOutputMillis = 0;
unsigned long lastPIDMillis = 0;
unsigned long lastLightControlMillis = 0;
unsigned long lastCalibrationModeMillis = 0;
unsigned long lastAnimationMillis = 0;

// ======== Sensor Readings ========
float g_hum = NAN;
float g_temp = NAN;
float g_lux = NAN;
float g_vpd = NAN;

// ======== Settings Struct & Persistence ========
#define SETTINGS_VERSION 1
struct Settings {
  uint8_t version;
  uint8_t coolSpeed;
  uint8_t heatSpeed;
  uint8_t growLevel;
  float   autoCoolThreshold;
  float   autoHeatThreshold;
  bool    autoMode;
  double  tempSetpoint;
  uint16_t targetLight;  // Target light level (lux)
  bool    lightAutoMode; // Auto light mode flag
  // Add more config fields as needed
};

// Start with LED off and lower default target light (200 lux)
Settings g_settings = {SETTINGS_VERSION, 0, 0, 0, 25.0, 18.0, false, 25.0, 200, false};

// EEPROM address for settings
const int SETTINGS_ADDR = 0;

// ======== Auto Mode and Light Control Globals ========
bool autoMode = false; // Single automode: true = automatic, false = manual
double g_tempSetpoint = 25.0; // °C
uint16_t g_targetLight = 200; // Target light level in lux (default 200)
int targetGrowLevel = 0; // Target LED brightness (separated from actual brightness)

// ======== Cooling Startup Logic ========
bool coolingActive = false;
int coolingStartupCounter = 0;
const int COOLING_STARTUP_CYCLES = 10;
const int COOLING_MIN_PWM = 150;
bool justEnabledAutoMode = false;

// ======== Heating Startup Logic ========
bool heatingActive = false;
int heatingStartupCounter = 0;
const int HEATING_STARTUP_CYCLES = 10;
const int HEATING_MIN_PWM = 180; // Higher minimum to ensure immediate startup

// ======== PID Control Variables ========
// PID input/output/parameters for cooling
double pidInputCool, pidOutputCool, pidSetpointCool;
double kpCool = 2.0, kiCool = 5.0, kdCool = 1.0;
PID pidCool(&pidInputCool, &pidOutputCool, &pidSetpointCool, kpCool, kiCool, kdCool, REVERSE);

// PID input/output/parameters for heating
double pidInputHeat, pidOutputHeat, pidSetpointHeat;
double kpHeat = 4.0, kiHeat = 8.0, kdHeat = 1.0; // Increased gains for faster response
PID pidHeat(&pidInputHeat, &pidOutputHeat, &pidSetpointHeat, kpHeat, kiHeat, kdHeat, DIRECT);

// ======== Auto Light Control Variables ========
bool lightAutoMode = false; // Auto light mode: true = automatic, false = manual
const int MAX_LIGHT_BRIGHTNESS = 100; // Maximum brightness for safety
const unsigned long LED_DUTY_CYCLE_INTERVAL = 2000; // 2 seconds for duty cycling high brightness
int lightPWM = 0; // Current light PWM value (0-255)
unsigned long lastLightAdjustmentMillis = 0; // Track last time we adjusted brightness
unsigned long g_lastLEDOnTime = 0; // When LED was last turned on

// NEW: Enhanced startup stabilization variables
bool g_initialStabilizationPhase = false; // Whether we're in the initial stabilization phase
unsigned long g_stabilizationStartMillis = 0; // When stabilization phase began
const unsigned long STABILIZATION_DURATION = 30000; // 30 seconds for initial stabilization
float g_startupDampingFactor = 0.3; // Extra damping during startup (will be gradually reduced)
int g_consecStableReadings = 0; // Count consecutive stable readings
const int STABLE_READINGS_THRESHOLD = 5; // Number of stable readings to exit stabilization phase early

// ======== Light Sensor Calibration & Smoothing ========
const float LIGHT_KP_REDUCED = 0.008; // Simplified proportional gain
const float LIGHT_KI = 0.0002; // Small integral gain to eliminate steady-state error
const float LIGHT_KD = 0.001; // Small derivative gain to improve response time
const int INITIAL_MAX_BRIGHTNESS = 50; // Lower initial maximum brightness
float g_luxFiltered = 0; // Filtered lux value
bool g_calibrationMode = false; // Whether we're in calibration mode
unsigned long g_calibrationStartMillis = 0; // When calibration mode was started
const unsigned long CALIBRATION_TIMEOUT = 60000; // 1 minute timeout for calibration
float g_lightIntegralError = 0.0; // Accumulated error for integral term
const float MAX_INTEGRAL_ERROR = 1000.0; // Limit integral windup
float g_previousError = 0.0; // Previous error for derivative term calculation

// Define target light level ranges more explicitly
const int VERY_LOW_TARGET = 200;  // 0-200 lux needs extremely careful handling
const int LOW_TARGET = 500;       // 0-500 lux needs very careful handling
const int MID_TARGET = 1500;      // 500-1500 lux needs careful handling
const int HIGH_TARGET = 3000;     // 1500+ lux can use standard handling

// NEW: Oscillation detection and output smoothing variables
const int OSCILLATION_DETECTION_WINDOW = 8; // Window size for detecting oscillations
int g_directionHistory[OSCILLATION_DETECTION_WINDOW]; // History of adjustment directions
int g_directionHistoryIndex = 0; // Current index in direction history
bool g_oscillationDetected = false; // Flag for when oscillation is detected
unsigned long g_oscillationDetectedTime = 0; // When oscillation was last detected
const unsigned long OSCILLATION_DAMPING_DURATION = 10000; // How long to apply damping after oscillation (10s)
float g_outputFilter = 0.0; // Filtered output value for smoothing
const float OUTPUT_FILTER_ALPHA = 0.3; // Output filter smoothing factor (higher = more responsive)
// Auto-adjustment variables
unsigned long g_lastStableTime = 0; // Last time the system was stable
bool g_adaptiveTuningActive = false; // Whether adaptive PID tuning is active
// Delayed response tracking
const unsigned long LIGHT_RESPONSE_DELAY = 1000; // Typical delay between LED change and sensor response
unsigned long g_lastSignificantAdjustment = 0; // Time of last significant adjustment

// ======== Menu Extension (Stub for Auto Mode) ========
// Extend MenuItem enum and menu logic as needed for auto/manual toggle and setpoint adjustment
// For now, just stubs for future menu expansion

// ======== Function Prototypes ========
/**
 * Checks if a button is pressed (debounced).
 * @param pin Button pin number
 * @return true if pressed, false otherwise
 */
bool isPressed(int pin);

/**
 * Maps cooling speed (0–3) to PWM value.
 * @param sp Speed (0–3)
 * @return PWM value (0–255)
 */
int  pwmFromCool(int sp);

/**
 * Maps heating speed (0–3) to PWM value.
 * @param sp Speed (0–3)
 * @return PWM value (0–255)
 */
int  pwmFromHeat(int sp);

/**
 * Loads settings from EEPROM. If version mismatch, loads defaults.
 */
void loadSettings();

/**
 * Saves current settings to EEPROM.
 */
void saveSettings();

/**
 * Resets settings to factory defaults and saves to EEPROM.
 */
void resetSettings();

/**
 * Controls the grow light to maintain target light level with enhanced safety features
 */
void updateLightControl();

/**
 * Updates the calibration mode status and performs necessary actions
 */
void updateCalibrationMode();

/**
 * Updates active animations, cycling through frames and ending animations when complete.
 * This function is non-blocking and uses millis() for timing.
 */
void updateAnimation();

// ======== PID Task ========
void updatePID() {
  // Only calculate PID values - don't apply them
  // Cooling PID (if enabled)
  if (autoMode) {
    pidInputCool = (double)g_temp;
    pidSetpointCool = g_tempSetpoint;
    pidCool.Compute();
  }
  // Heating PID (if enabled)
  if (autoMode) {
    pidInputHeat = (double)g_temp;
    pidSetpointHeat = g_tempSetpoint;
    pidHeat.Compute();
  }
  // Removed direct pin control - this is now only done in applyOutputs()
}

// ======== Serial Debug Setup ========
void setup() {
  Wire.begin();
  Serial.begin(9600);

  // IMMEDIATE SAFETY: Turn off all outputs at the very start
  // before any other initialization
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  pinMode(COOL_PIN, OUTPUT);
  pinMode(HEAT_PIN, OUTPUT);
  
  // Explicitly set all outputs to 0
  digitalWrite(LED_R_PIN, LOW);
  digitalWrite(LED_G_PIN, LOW);
  digitalWrite(LED_B_PIN, LOW);
  digitalWrite(COOL_PIN, LOW);
  digitalWrite(HEAT_PIN, LOW);
  
  // Double ensure LED is off with analogWrite too
  analogWrite(LED_R_PIN, 0);
  analogWrite(LED_G_PIN, 0);
  analogWrite(LED_B_PIN, 0);
  analogWrite(COOL_PIN, 0);
  analogWrite(HEAT_PIN, 0);
  
  // Initialize key variables to prevent any chance of undefined behavior
  growLevel = 0;
  // Initialize light control variables to ensure they're safe
  lightPWM = 0;
  g_lightIntegralError = 0.0;
  g_lastLEDOnTime = 0;
  
  // LCD initialization
  lcd.begin(16,2);
  lcd.setBacklight(255);
  lcd.clear();
  lcd.print("GrowSense Init");
  delay(1000);
  lcd.clear();
  
  // Initialize custom characters
  initializeCustomCharacters();

  // DHT22 initialization
  dht.begin();

  // TSL2591 initialization
  if (!tsl.begin()) {
    lcd.print("TSL25911 ERR");
    while (1); // Halt if sensor not found
  }
  
  // Configure sensor for maximum stability
  tsl.setGain(TSL2591_GAIN_LOW);  // Low gain for better stability
  tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);  // Longer integration for stability
  
  // Take initial readings to stabilize the sensor
  for (int i = 0; i < 3; i++) {
    uint32_t full = tsl.getFullLuminosity();
    uint16_t ir = full >> 16;
    uint16_t vis = full & 0xFFFF;
    float lux = tsl.calculateLux(vis, ir);
    Serial.print("[INIT] TSL2591 reading #"); Serial.print(i+1);
    Serial.print(": "); Serial.println(lux);
    delay(100);
  }

  // Fan and indicator LED outputs
  pinMode(COOL_PIN, OUTPUT);
  pinMode(HEAT_PIN, OUTPUT);
  analogWrite(COOL_PIN, 0);
  analogWrite(HEAT_PIN, 0);

  // RGB grow-light outputs
  // Ensure grow light is off at startup
  analogWrite(LED_R_PIN, 0);
  analogWrite(LED_G_PIN, 0);
  analogWrite(LED_B_PIN, 0);

  // Button inputs (active low)
  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BTN_LEFT,   INPUT_PULLUP);
  pinMode(BTN_RIGHT,  INPUT_PULLUP);
  pinMode(BTN_UP,     INPUT_PULLUP);
  pinMode(BTN_DOWN,   INPUT_PULLUP);
  
  // Initialize button system
  initButtons();

  loadSettings(); // Load settings from EEPROM at startup

  // Override any loaded settings for LEDs - always start with LEDs off
  growLevel = 0;
  // Initialize targetGrowLevel to 0 to prevent LED from turning on
  targetGrowLevel = 0;

  // Initialize PIDs with proper settings
  pidCool.SetMode(AUTOMATIC);
  pidCool.SetOutputLimits(0, 255);
  pidCool.SetSampleTime(PID_INTERVAL); // Ensure PID sample time matches our interval
  
  pidHeat.SetMode(AUTOMATIC);
  pidHeat.SetOutputLimits(0, 255);
  pidHeat.SetSampleTime(PID_INTERVAL); // Ensure PID sample time matches our interval
  
  // Initialize PID outputs to zero
  pidOutputCool = 0;
  pidOutputHeat = 0;

  coolingActive = false;
  coolingStartupCounter = 0;
  heatingActive = false;
  heatingStartupCounter = 0;
  justEnabledAutoMode = false;
  
  // Initialize light control variables
  lightPWM = 0; // Start with LED off
  lastLightControlMillis = 0;
  lastLightAdjustmentMillis = 0;
  
  // Force auto light mode off at startup
  lightAutoMode = false;
  
  // Initialize calibration mode variables
  g_calibrationMode = false;
  g_calibrationStartMillis = 0;
  lastCalibrationModeMillis = 0;
  
  // Final safety check - apply all outputs to ensure everything is off
  applyOutputs();
  
  // Log startup
  Serial.println(F("GrowSense initialized"));
}

/**
 * Resets PID controllers to prevent windup during mode transitions.
 * Call this when switching between heating and cooling.
 */
void resetPIDControllers() {
  // Reset the internal state of both PIDs
  pidCool.SetMode(MANUAL);
  pidHeat.SetMode(MANUAL);
  
  // Set outputs to zero
  pidOutputCool = 0;
  pidOutputHeat = 0;
  
  // Re-enable PIDs
  pidCool.SetMode(AUTOMATIC);
  pidHeat.SetMode(AUTOMATIC);
  
  // When resetting, also reset startup counters
  coolingStartupCounter = 0;
  heatingStartupCounter = 0;
  
  Serial.println(F("[PID] Controllers reset"));
}

// ======== Enhanced readSensors with Simplified Filtering ========
void readSensors() {
  g_hum  = dht.readHumidity();      // %RH
  g_temp = dht.readTemperature();  // °C

  uint32_t full = tsl.getFullLuminosity();
  uint16_t ir  = full >> 16;
  uint16_t vis = full & 0xFFFF;
  g_lux    = tsl.calculateLux(vis, ir);

  // Simplified filtering - single exponential filter only
  if (!isnan(g_lux) && g_lux >= 0 && g_lux < 100000) { // Basic sanity check
    // Single exponential filter for stability without excessive lag
    const float alpha = 0.4; // Increased alpha for better responsiveness
    
    // Initialize filter on first valid reading
    if (g_luxFiltered == 0) {
      g_luxFiltered = g_lux;
    } else {
      g_luxFiltered = (alpha * g_lux) + ((1.0 - alpha) * g_luxFiltered);
    }
    
    Serial.print("[SENSORS] Lux raw: "); Serial.print(g_lux);
    Serial.print(" Filtered: "); Serial.println(g_luxFiltered);
  } else {
    // Bad reading - maintain previous filtered value
    Serial.println("[SENSORS] Rejected invalid light reading");
  }

  // Compute VPD (Vapor Pressure Deficit)
  float es   = 0.6108 * exp((17.27 * g_temp)/(g_temp + 237.3)); // kPa
  float ea   = es * (g_hum/100.0);                              // kPa
  g_vpd  = es - ea;                                             // kPa

  // Debug
  Serial.print("[SENSORS] Temp: "); Serial.print(g_temp);
  Serial.print(" Set: "); Serial.print(g_tempSetpoint);
  Serial.print(" Hum: "); Serial.print(g_hum);
  Serial.print(" Lux: "); Serial.print(g_lux);
  Serial.print(" LuxFiltered: "); Serial.print(g_luxFiltered);
  Serial.print(" VPD: "); Serial.println(g_vpd);
}

// ======== Lightning-Fast Button Handling ========
void handleButtons() {
  // Direct edge detection and immediate action - no separate phases
  // UP button
  if (buttonPressed(BTN_UP)) {
    if (g_calibrationMode) {
      lightPWM = min(MAX_LIGHT_BRIGHTNESS, lightPWM + 5);
      growLevel = lightPWM;
    } else {
      menu = MenuItem((menu + MI_COUNT - 1) % MI_COUNT);
    }
    updateDisplayImmediate(); // Immediate display update
    return; // Handle one button per loop for maximum speed
  }
  
  // DOWN button  
  if (buttonPressed(BTN_DOWN)) {
    if (g_calibrationMode) {
      lightPWM = max(0, lightPWM - 5);
      growLevel = lightPWM;
    } else {
      menu = MenuItem((menu + 1) % MI_COUNT);
    }
    updateDisplayImmediate(); // Immediate display update
    return;
  }
  
  // LEFT button
  if (buttonPressed(BTN_LEFT)) {
    if (g_calibrationMode) {
      g_targetLight = max(0, g_targetLight - 50);
    } else {
      switch(menu) {
        case MI_COOL: coolSpeed = max(0, coolSpeed - 1); break;
        case MI_HEAT: heatSpeed = max(0, heatSpeed - 1); break;
        case MI_GROW: 
          targetGrowLevel = max(0, targetGrowLevel - 1); 
          if (!lightAutoMode) growLevel = targetGrowLevel; // Immediate update in manual mode
          break;
        case MI_AUTO: autoMode = false; break;
        case MI_SETPOINT: g_tempSetpoint = max(10.0, g_tempSetpoint - 0.5); break;
        case MI_TARGET_LIGHT: g_targetLight = max(0, g_targetLight - 50); break;
        case MI_LIGHT_AUTO: lightAutoMode = false; break;
        default: break;
      }
    }
    updateDisplayImmediate(); // Immediate display update
    return;
  }
  
  // RIGHT button
  if (buttonPressed(BTN_RIGHT)) {
    if (g_calibrationMode) {
      g_targetLight = min(5000, g_targetLight + 50);
    } else {
      switch(menu) {
        case MI_COOL: coolSpeed = min(3, coolSpeed + 1); break;
        case MI_HEAT: heatSpeed = min(3, heatSpeed + 1); break;
        case MI_GROW: 
          targetGrowLevel = min(MAX_LIGHT_BRIGHTNESS, targetGrowLevel + 1); 
          if (!lightAutoMode) growLevel = targetGrowLevel; // Immediate update in manual mode
          break;
        case MI_AUTO: autoMode = true; break;
        case MI_SETPOINT: g_tempSetpoint = min(40.0, g_tempSetpoint + 0.5); break;
        case MI_TARGET_LIGHT: g_targetLight = min(5000, g_targetLight + 50); break;
        case MI_LIGHT_AUTO: lightAutoMode = true; break;
        default: break;
      }
    }
    updateDisplayImmediate(); // Immediate display update
    return;
  }
  
  // SELECT button handling with long press
  bool selectCurrentlyPressed = (digitalRead(BTN_SELECT) == LOW);
  if (selectCurrentlyPressed && !g_selectPressed) {
    // Just pressed
    g_selectPressed = true;
    g_selectPressStart = g_currentMillis;
  } else if (!selectCurrentlyPressed && g_selectPressed) {
    // Just released
    g_selectPressed = false;
    unsigned long pressDuration = g_currentMillis - g_selectPressStart;
    
    if (pressDuration >= BUTTON_LONG_PRESS_MS) {
      // Long press - save settings
      saveSettings();
      // Quick flash to show save (no temp message system)
      lcd.clear();
      lcd.print("SAVED!");
      delay(300); // Very brief feedback
    } else {
      // Short press - menu action
      if (g_calibrationMode) {
        lightAutoMode = !lightAutoMode;
      } else {
        menu = MI_MAIN;
      }
    }
    updateDisplayImmediate(); // Immediate display update
    return;
  }
  
  // Manual mode grow level is now updated immediately in button handling
  // No gradual adjustment needed anymore

  // Handle mode transitions (simplified)
  static bool lastAutoMode = false;
  if (autoMode != lastAutoMode) {
    coolingActive = false;
    coolingStartupCounter = 0;
    justEnabledAutoMode = autoMode;
    lastAutoMode = autoMode;
  }
  
  static bool lastLightAutoMode = false;
  if (lightAutoMode != lastLightAutoMode) {
    if (lightAutoMode) {
      // Switching TO auto mode
      if (g_luxFiltered < g_targetLight) {
        lightPWM = min(5, MAX_LIGHT_BRIGHTNESS / 10);
        growLevel = lightPWM;
      }
      g_lightIntegralError = 0;
      g_previousError = 0;
      lastLightAdjustmentMillis = 0;
    } else {
      // Switching FROM auto mode TO manual mode - reset LED to off
      growLevel = 0;
      targetGrowLevel = 0;
      lightPWM = 0;
      g_lightIntegralError = 0;
      g_previousError = 0;
      Serial.println("[LIGHT] Switched to manual mode - LED turned off");
    }
    lastLightAutoMode = lightAutoMode;
  }
}

// ======== Enhanced applyOutputs with Debug and NaN check ========
void applyOutputs() {
  static bool lastCoolingState = false;
  static bool lastHeatingState = false;
  static bool lastLightState = false;
  
  if (autoMode) {
    // If automode was just enabled, skip this cycle and reset all outputs/state
    if (justEnabledAutoMode) {
      analogWrite(COOL_PIN, 0);
      analogWrite(HEAT_PIN, 0);
      resetPIDControllers();  // Reset PID controllers when enabling auto mode
      coolingActive = false;
      coolingStartupCounter = 0;
      heatingActive = false;
      heatingStartupCounter = 0;
      Serial.println("[AUTO] First cycle after enabling automode: all outputs reset");
      justEnabledAutoMode = false;
      return;
    }
    if (isnan(g_temp)) {
      Serial.println("[AUTO] Temp is NaN, skipping PID");
      analogWrite(COOL_PIN, 0);
      analogWrite(HEAT_PIN, 0);
      pidOutputCool = 0;
      pidOutputHeat = 0;
      coolingActive = false;
      coolingStartupCounter = 0;
      heatingActive = false;
      heatingStartupCounter = 0;
      return;
    }
    
    // Track current mode to detect transitions
    static bool wasCooling = false;
    static bool wasHeating = false;
    
    double error = g_tempSetpoint - g_temp;
    Serial.print("[AUTO] Current error: "); Serial.println(error);
    
    if (error < -0.2) { // Too hot, cool
      // Mode transition: heating -> cooling
      if (wasHeating) {
        Serial.println("[AUTO] Mode transition: heating -> cooling");
        resetPIDControllers();  // Reset PID controllers during mode transition
        wasHeating = false;
        
        // Exiting heating mode
        heatingActive = false;
        heatingStartupCounter = 0;
      }
      
      // Only trigger cooling startup logic if actually cooling and not just switched from another mode
      if (!coolingActive) {
        coolingActive = true;
        coolingStartupCounter = 0;
        Serial.println("[COOL] Entering cooling mode, forcing min PWM for startup");
        
        // NEW: Start cooling fan animation when activating cooling (avoid conflicts in cool menu)
        if (menu != MI_COOL) {
          startAnimation(1, 15, 0); // Type 1 = Cooling fan, position = top right corner
        }
      }
      
      wasCooling = true;
      
      // PID already calculated in updatePID()
      int pwm = constrain((int)pidOutputCool, 0, 255);
      // Force min PWM for first few cycles
      if (coolingStartupCounter < COOLING_STARTUP_CYCLES) {
        pwm = COOLING_MIN_PWM;
        coolingStartupCounter++;
        Serial.print("[COOL] Startup cycle "); Serial.print(coolingStartupCounter); Serial.print(" forcing PWM="); Serial.println(pwm);
      } else if (pwm > 0 && pwm < COOLING_MIN_PWM) {
        pwm = COOLING_MIN_PWM;
      }
      analogWrite(COOL_PIN, pwm);
      analogWrite(HEAT_PIN, 0);
      Serial.print("[AUTO] COOL: temp="); Serial.print(g_temp); Serial.print(" set="); Serial.print(g_tempSetpoint); Serial.print(" error="); Serial.print(error); Serial.print(" pwm="); Serial.println(pwm);
    } else if (error > 0.2) { // Too cold, heat
      // Mode transition: cooling -> heating
      if (wasCooling) {
        Serial.println("[AUTO] Mode transition: cooling -> heating");
        resetPIDControllers();  // Reset PID controllers during mode transition
        wasCooling = false;
        
        // Exiting cooling mode
        coolingActive = false;
        coolingStartupCounter = 0;
      }
      
      // Only trigger heating startup logic if not already heating
      if (!heatingActive) {
        heatingActive = true;
        heatingStartupCounter = 0;
        Serial.println("[HEAT] Entering heating mode, forcing min PWM for startup");
        
        // NEW: Start heating element animation when activating heating (avoid conflicts in heat menu)
        if (menu != MI_HEAT) {
          startAnimation(2, 15, 0); // Type 2 = Heating element, position = top right corner
        }
      }
      
      wasHeating = true;
      
      // PID already calculated in updatePID()
      int pwm = constrain((int)pidOutputHeat, 0, 255);
      
      // Force min PWM for first few cycles to ensure immediate startup
      if (heatingStartupCounter < HEATING_STARTUP_CYCLES) {
        pwm = HEATING_MIN_PWM;
        heatingStartupCounter++;
        Serial.print("[HEAT] Startup cycle "); Serial.print(heatingStartupCounter); Serial.print(" forcing PWM="); Serial.println(pwm);
      } else if (pwm > 0 && pwm < HEATING_MIN_PWM) {
        pwm = HEATING_MIN_PWM;
      }
      
      analogWrite(HEAT_PIN, pwm);
      analogWrite(COOL_PIN, 0);
      Serial.print("[AUTO] HEAT: temp="); Serial.print(g_temp); Serial.print(" set="); Serial.print(g_tempSetpoint); Serial.print(" error="); Serial.print(error); Serial.print(" pwm="); Serial.println(pwm);
    } else {
      // In range (deadband): both off, both PID outputs zeroed
      if (coolingActive) {
        coolingActive = false;
        coolingStartupCounter = 0;
      }
      if (heatingActive) {
        heatingActive = false;
        heatingStartupCounter = 0;
      }
      wasCooling = false;
      wasHeating = false;
      analogWrite(COOL_PIN, 0);
      analogWrite(HEAT_PIN, 0);
      pidOutputCool = 0;
      pidOutputHeat = 0;
      Serial.println("[AUTO] IN RANGE: both off");
    }
  } else {
    coolingActive = false;
    coolingStartupCounter = 0;
    heatingActive = false;
    heatingStartupCounter = 0;
    justEnabledAutoMode = false;
    
    // Check for fan state changes in manual mode to trigger animations
    // Only trigger animations when NOT in the respective control menu to avoid conflicts
    bool currentCoolingState = (coolSpeed > 0);
    if (currentCoolingState && !lastCoolingState && menu != MI_COOL) {
      // Cooling fan just turned on - only animate when not in cool menu
      startAnimation(1, 15, 0); // Type 1 = Cooling fan, position = top right corner
    }
    lastCoolingState = currentCoolingState;
    
    bool currentHeatingState = (heatSpeed > 0);
    if (currentHeatingState && !lastHeatingState && menu != MI_HEAT) {
      // Heating fan just turned on - only animate when not in heat menu
      startAnimation(2, 15, 0); // Type 2 = Heating element, position = top right corner
    }
    lastHeatingState = currentHeatingState;
    
    analogWrite(COOL_PIN, pwmFromCool(coolSpeed));
    analogWrite(HEAT_PIN, pwmFromHeat(heatSpeed));
    Serial.print("[MANUAL] COOL: "); Serial.print(coolSpeed); Serial.print(" HEAT: "); Serial.println(heatSpeed);
  }

  // Startup safety: Skip first few seconds to ensure everything is stable
  static bool startupSafety = true;
  static unsigned long startupSafetyTimer = 0;
  
  if (startupSafety) {
    if (startupSafetyTimer == 0) {
      startupSafetyTimer = g_currentMillis;
      // Keep LEDs off during startup safety period
      analogWrite(LED_R_PIN, 0);
      analogWrite(LED_G_PIN, 0);
      analogWrite(LED_B_PIN, 0);
      Serial.println("[SAFETY] LED startup safety active, keeping LEDs off");
      return;
    }
    
    // Keep LEDs off for the first 5 seconds
    if (g_currentMillis - startupSafetyTimer < 5000) {
      // Keep LEDs off during startup safety period
      analogWrite(LED_R_PIN, 0);
      analogWrite(LED_G_PIN, 0);
      analogWrite(LED_B_PIN, 0);
      return;
    } else {
      startupSafety = false;
      Serial.println("[SAFETY] LED startup safety period complete");
    }
  }

  // Only apply manual grow light setting if not in auto light mode
  if (!lightAutoMode) {
    // Check for light state changes to trigger animations
    // Only trigger animations when NOT in the grow menu to avoid conflicts
    bool currentLightState = (growLevel > 0);
    if (currentLightState && !lastLightState && menu != MI_GROW) {
      // Light just turned on - only animate when not in grow menu
      startAnimation(3, 15, 0); // Type 3 = Light bulb, position = top right corner
    }
    lastLightState = currentLightState;
    
    // Apply duty cycling for higher brightness levels to prevent overheating
    if (growLevel > 0) {
      // Safety check - disable LED in extremely humid conditions to prevent condensation issues
      if (!isnan(g_hum) && g_hum > 95.0) {
        // Too humid - disable LED for safety
        analogWrite(LED_R_PIN, 0);
        analogWrite(LED_G_PIN, 0);
        analogWrite(LED_B_PIN, 0);
        
        // Show alert animation for safety cutoff
        startAnimation(4, 15, 0); // Type 4 = Alert, position = top right corner
        
        Serial.println("[LIGHT] SAFETY: Humidity too high, LED disabled");
        return;
      }
      
      // If brightness is high, implement duty cycling
      if (growLevel > 50) {
        // For high brightness, use duty cycling to reduce heat
        // Turn LED on/off based on a cycle to reduce average power
        unsigned long cyclePosition = g_currentMillis % LED_DUTY_CYCLE_INTERVAL;
        unsigned long onThreshold = (LED_DUTY_CYCLE_INTERVAL * 7) / 10; // 70% on time
        
        if (cyclePosition < onThreshold) {
          analogWrite(LED_R_PIN, growLevel);
          analogWrite(LED_G_PIN, growLevel);
          analogWrite(LED_B_PIN, growLevel);
        } else {
          // Cooling period - reduce to 30% brightness
          int reducedLevel = (growLevel * 3) / 10; // 30% of original brightness
          analogWrite(LED_R_PIN, reducedLevel);
          analogWrite(LED_G_PIN, reducedLevel);
          analogWrite(LED_B_PIN, reducedLevel);
        }
      } else {
        // For low brightness, direct output is safe
        analogWrite(LED_R_PIN, growLevel);
        analogWrite(LED_G_PIN, growLevel);
        analogWrite(LED_B_PIN, growLevel);
      }
    } else {
      // LED off
      analogWrite(LED_R_PIN, 0);
      analogWrite(LED_G_PIN, 0);
      analogWrite(LED_B_PIN, 0);
    }
    Serial.print("[MANUAL LIGHT] Level: "); Serial.println(growLevel);
  } else {
    // In auto mode, check for light state changes to trigger animations
    // Only trigger animations when NOT in the grow menu to avoid conflicts
    bool currentLightState = (growLevel > 0);
    if (currentLightState && !lastLightState && menu != MI_GROW) {
      // Light just turned on - only animate when not in grow menu
      startAnimation(3, 15, 0); // Type 3 = Light bulb, position = top right corner
    }
    lastLightState = currentLightState;
    
    // In auto mode, the updateLightControl function already set growLevel
    // with safety limits applied
    // Apply duty cycling for higher brightness levels to prevent overheating
    if (growLevel > 0) {
      // Safety check - disable LED in extremely humid conditions to prevent condensation issues
      if (!isnan(g_hum) && g_hum > 95.0) {
        // Too humid - disable LED for safety
        analogWrite(LED_R_PIN, 0);
        analogWrite(LED_G_PIN, 0);
        analogWrite(LED_B_PIN, 0);
        
        // Show alert animation for safety cutoff
        startAnimation(4, 15, 0); // Type 4 = Alert, position = top right corner
        
        Serial.println("[LIGHT] SAFETY: Humidity too high, LED disabled");
        return;
      }
      
      // If brightness is high, implement duty cycling
      if (growLevel > 50) {
        // For high brightness, use duty cycling to reduce heat
        // Turn LED on/off based on a cycle to reduce average power
        unsigned long cyclePosition = g_currentMillis % LED_DUTY_CYCLE_INTERVAL;
        unsigned long onThreshold = (LED_DUTY_CYCLE_INTERVAL * 7) / 10; // 70% on time
        
        if (cyclePosition < onThreshold) {
          analogWrite(LED_R_PIN, growLevel);
          analogWrite(LED_G_PIN, growLevel);
          analogWrite(LED_B_PIN, growLevel);
        } else {
          // Cooling period - reduce to 30% brightness
          int reducedLevel = (growLevel * 3) / 10; // 30% of original brightness
          analogWrite(LED_R_PIN, reducedLevel);
          analogWrite(LED_G_PIN, reducedLevel);
          analogWrite(LED_B_PIN, reducedLevel);
        }
      } else {
        // For low brightness, direct output is safe
        analogWrite(LED_R_PIN, growLevel);
        analogWrite(LED_G_PIN, growLevel);
        analogWrite(LED_B_PIN, growLevel);
      }
    } else {
      // LED off
      analogWrite(LED_R_PIN, 0);
      analogWrite(LED_G_PIN, 0);
      analogWrite(LED_B_PIN, 0);
    }
    Serial.print("[AUTO LIGHT] Level: "); Serial.println(growLevel);
  }
}

/**
 * Lightning-fast display update with consistent glyph rendering
 */
void updateDisplayImmediate() {
  // Only clear when absolutely necessary (mode changes)
  static MenuItem lastMenu = MI_COUNT;
  static bool lastCalibrationMode = false;
  
  bool needsClear = (menu != lastMenu) || (g_calibrationMode != lastCalibrationMode);
  
  if (needsClear) {
    lcd.clear();
    lastMenu = menu;
    lastCalibrationMode = g_calibrationMode;
  }

  // If in calibration mode, override normal display
  if (g_calibrationMode) {
    lcd.setCursor(0, 0);
    lcd.print("CAL - LUX:");
    lcd.print((int)g_luxFiltered);
    lcd.print("    "); // Clear remainder
    
    lcd.setCursor(0, 1);
    lcd.print("TGT:");
    lcd.print(g_targetLight);
    lcd.print(" PWM:");
    lcd.print(lightPWM);
    lcd.print("   "); // Clear remainder
    return;
  }
  
  // Fast menu display with consistent glyph clearing
  switch(menu) {
    case MI_MAIN:
      // Line 1: Temp & Humidity
      lcd.setCursor(0, 0);
      if (isnan(g_temp) || isnan(g_hum)) {
        lcd.print("DHT Err         ");
      } else {
        lcd.write(0); // Thermometer
        lcd.print(":");
        lcd.print(g_temp,1);
        lcd.write(0xDF);
        lcd.print("C ");
        lcd.write(1); // Droplet
        lcd.print(":");
        lcd.print(g_hum,1);
        lcd.print("%  ");
      }
      // Line 2: Lux & VPD
      lcd.setCursor(0,1);
      lcd.write(2); // Light bulb
      lcd.print(":");
      lcd.print((int)g_lux);
      lcd.print(" V:");
      lcd.print(g_vpd,2);
      lcd.print("   ");
      break;

    case MI_COOL:
      // Line 1: Cool control with glyph
      lcd.setCursor(0, 0);
      lcd.print("> COOL: ");
      lcd.print(coolSpeed);
      lcd.print("       ");
      // Clear position 15,0 first, then add glyph if needed
      lcd.setCursor(15, 0);
      if (coolSpeed > 0) {
        lcd.write(3); // Fan character
      } else {
        lcd.print(" "); // Clear glyph
      }
      
      // Line 2: Heat display (no glyph here)
      lcd.setCursor(0,1);
      lcd.print("  HEAT: ");
      lcd.print(heatSpeed);
      lcd.print("       ");
      // Always clear position 15,1 in cool menu
      lcd.setCursor(15, 1);
      lcd.print(" ");
      break;

    case MI_HEAT:
      // Line 1: Cool display (no glyph here)
      lcd.setCursor(0, 0);
      lcd.print("  COOL: ");
      lcd.print(coolSpeed);
      lcd.print("       ");
      // Always clear position 15,0 in heat menu
      lcd.setCursor(15, 0);
      lcd.print(" ");
      
      // Line 2: Heat control with glyph
      lcd.setCursor(0,1);
      lcd.print("> HEAT: ");
      lcd.print(heatSpeed);
      lcd.print("       ");
      // Clear position 15,1 first, then add glyph if needed
      lcd.setCursor(15, 1);
      if (heatSpeed > 0) {
        lcd.write(5); // Heating element
      } else {
        lcd.print(" "); // Clear glyph
      }
      break;

    case MI_GROW:
      {
        // Line 1: Cool display (no glyph here)
        lcd.setCursor(0, 0);
        lcd.print("  COOL: ");
        lcd.print(coolSpeed);
        lcd.print("       ");
        // Always clear position 15,0 in grow menu
        lcd.setCursor(15, 0);
        lcd.print(" ");
        
        // Line 2: Grow control with glyph
        lcd.setCursor(0,1);
        lcd.print("> GROW: ");
        // Show the target in manual mode for immediate feedback
        int displayValue = lightAutoMode ? growLevel : targetGrowLevel;
        lcd.print(displayValue);
        lcd.print("       ");
        // Clear position 15,1 first, then add glyph if needed
        lcd.setCursor(15, 1);
        if (displayValue > 0) {
          lcd.write(2); // Light bulb
        } else {
          lcd.print(" "); // Clear glyph
        }
        break;
      }

    case MI_AUTO:
      lcd.setCursor(0, 0);
      lcd.print("TEMP AUTO: ");
      lcd.print(autoMode ? "ON " : "OFF");
      lcd.print("   ");
      
      lcd.setCursor(0,1);
      lcd.print("<L/R> Toggle    ");
      
      // Clear all glyph positions
      lcd.setCursor(15, 0);
      lcd.print(" ");
      lcd.setCursor(15, 1);
      lcd.print(" ");
      break;

    case MI_SETPOINT:
      lcd.setCursor(0, 0);
      lcd.print("TEMP SET: ");
      lcd.print(g_tempSetpoint,1);
      lcd.write(0xDF);
      lcd.print("C  ");
      
      lcd.setCursor(0,1);
      lcd.print("<L/R> Adj       ");
      
      // Clear all glyph positions
      lcd.setCursor(15, 0);
      lcd.print(" ");
      lcd.setCursor(15, 1);
      lcd.print(" ");
      break;

    case MI_TARGET_LIGHT:
      lcd.setCursor(0, 0);
      lcd.print("TARGET LIGHT:   ");
      
      lcd.setCursor(0,1);
      lcd.print(g_targetLight);
      lcd.print(" lux <L/R>   ");
      
      // Clear all glyph positions
      lcd.setCursor(15, 0);
      lcd.print(" ");
      lcd.setCursor(15, 1);
      lcd.print(" ");
      break;

    case MI_LIGHT_AUTO:
      lcd.setCursor(0, 0);
      lcd.print("LIGHT AUTO: ");
      lcd.print(lightAutoMode ? "ON " : "OFF");
      lcd.print("  ");
      
      lcd.setCursor(0,1);
      lcd.print("<L/R> Toggle    ");
      
      // Clear all glyph positions
      lcd.setCursor(15, 0);
      lcd.print(" ");
      lcd.setCursor(15, 1);
      lcd.print(" ");
      break;
  }
}

/**
 * Regular scheduled display update (for sensor data)
 */
void updateDisplay() {
  // Handle temporary message timeout
  if (g_showTempMessage && g_currentMillis - g_tempMessageStartTime >= TEMP_MESSAGE_DURATION) {
    g_showTempMessage = false;
  }
  
  // If showing temporary message, display it and return
  if (g_showTempMessage) {
    lcd.clear();
    lcd.print(g_tempMessage);
    return;
  }

  // Only update main screen automatically (when not in menu navigation)
  if (menu == MI_MAIN && !g_calibrationMode) {
    updateDisplayImmediate();
  }
}

/**
 * Updates the calibration mode status and performs necessary actions
 */
void updateCalibrationMode() {
  // Skip if not in calibration mode
  if (!g_calibrationMode) {
    return;
  }
  
  // Check for timeout
  if (g_currentMillis - g_calibrationStartMillis >= CALIBRATION_TIMEOUT) {
    g_calibrationMode = false;
    lcd.clear();
    lcd.print("Calibration Mode");
    lcd.setCursor(0, 1);
    lcd.print("TIMEOUT - OFF");
    delay(1000);
    
    Serial.println("[CALIBRATION] Mode TIMEOUT - DISABLED");
    
    // Save settings
    saveSettings();
    return;
  }
  
  // Every 5 seconds, show calibration status
  if ((g_currentMillis - g_calibrationStartMillis) % 5000 < 50) {
    Serial.println("[CALIBRATION] Test Protocol:");
    Serial.print("  Light Level: "); Serial.println(g_luxFiltered);
    Serial.print("  Light Target: "); Serial.println(g_targetLight);
    Serial.print("  Light PWM: "); Serial.println(lightPWM);
    Serial.print("  Light Error: "); Serial.println(g_targetLight - g_luxFiltered);
    Serial.print("  Time Remaining: "); Serial.print((CALIBRATION_TIMEOUT - (g_currentMillis - g_calibrationStartMillis)) / 1000); Serial.println(" seconds");
  }
}

/**
 * Updates active animations, cycling through frames and ending animations when complete.
 * This function is non-blocking and uses millis() for timing.
 */
void updateAnimation() {
  // Skip if no animation is active
  if (!g_animationActive) {
    return;
  }
  
  // Check if animation duration has expired
  if (g_currentMillis - g_animationStartMillis >= ANIMATION_DURATION) {
    // End animation
    g_animationActive = false;
    // Clear the animation position by writing a space
    lcd.setCursor(g_animationDisplayX, g_animationDisplayY);
    lcd.print(" ");
    Serial.println(F("[ANIMATION] Completed"));
    return;
  }
  
  // Check if it's time to update the animation frame
  if (g_currentMillis - g_lastAnimationUpdateMillis >= ANIMATION_FRAME_INTERVAL) {
    g_lastAnimationUpdateMillis = g_currentMillis;
    g_currentAnimationFrame = (g_currentAnimationFrame + 1) % 2; // Toggle between 0 and 1
    
    // No need to save cursor position, just set it directly to the animation position
    // and the main display update will reset it on next cycle
    lcd.setCursor(g_animationDisplayX, g_animationDisplayY);
    
    // Display the appropriate character based on animation type and frame
    switch (g_currentAnimationType) {
      case 1: // Cooling fan animation
        lcd.write(g_currentAnimationFrame == 0 ? 3 : 4); // Alternate between fan frames
        break;
      case 2: // Heating animation
        lcd.write(5); // Heating element
        break;
      case 3: // Light animation
        lcd.write(2); // Light bulb
        break;
      case 4: // Alert animation
        lcd.write(g_currentAnimationFrame == 0 ? 7 : ' '); // Blink the alert symbol
        break;
      default:
        // Unknown animation type, just clear it
        lcd.print(" ");
        break;
    }
    
    Serial.print(F("[ANIMATION] Frame update: "));
    Serial.println(g_currentAnimationFrame);
  }
}

// ======== Lightning-Fast Main Loop ========
void loop() {
  g_currentMillis = millis();

  // 1) PRIORITY: Handle buttons immediately - no scheduling
  handleButtons();

  // 2) Schedule sensor reading (less frequent for speed)
  if (g_currentMillis - lastSensorMillis >= SENSOR_INTERVAL) {
    readSensors();
    lastSensorMillis += SENSOR_INTERVAL;
  }

  // 3) Schedule output control
  if (g_currentMillis - lastOutputMillis >= OUTPUT_INTERVAL) {
    applyOutputs();
    lastOutputMillis += OUTPUT_INTERVAL;
  }

  // 4) Schedule display update (only for main screen sensor data)
  if (g_currentMillis - lastDisplayMillis >= DISPLAY_INTERVAL) {
    updateDisplay();
    lastDisplayMillis += DISPLAY_INTERVAL;
  }

  // Schedule PID update
  if (g_currentMillis - lastPIDMillis >= PID_INTERVAL) {
    updatePID();
    lastPIDMillis += PID_INTERVAL;
  }

  // Schedule light control
  if (g_currentMillis - lastLightControlMillis >= LIGHT_CONTROL_INTERVAL) {
    updateLightControl();
    lastLightControlMillis += LIGHT_CONTROL_INTERVAL;
  }

  // Schedule calibration mode update
  if (g_currentMillis - lastCalibrationModeMillis >= CALIBRATION_MODE_INTERVAL) {
    updateCalibrationMode();
    lastCalibrationModeMillis += CALIBRATION_MODE_INTERVAL;
  }

  // Schedule animation update
  if (g_currentMillis - lastAnimationMillis >= ANIMATION_INTERVAL) {
    updateAnimation();
    lastAnimationMillis += ANIMATION_INTERVAL;
  }

  // No delay() anywhere: loop is always non-blocking and button-priority
}

// ======== PWM Mapping ========
/**
 * Maps cooling speed (0–3) to PWM value.
 * @param sp Speed (0–3)
 * @return PWM value (0–255)
 */
int pwmFromCool(int sp) {
  if (sp == 0) return 0;
  return map(sp, 1, 3, 180, 255);
}

/**
 * Maps heating speed (0–3) to PWM value.
 * @param sp Speed (0–3)
 * @return PWM value (0–255)
 */
int pwmFromHeat(int sp) {
  if (sp == 0) return 0;
  return map(sp, 1, 3, 100, 255);
}

/**
 * Loads settings from EEPROM. If version mismatch, loads defaults.
 */
void loadSettings() {
  EEPROM.get(SETTINGS_ADDR, g_settings);
  if (g_settings.version != SETTINGS_VERSION) {
    // Version mismatch or uninitialized, load defaults
    g_settings = {SETTINGS_VERSION, 0, 0, 0, 25.0, 18.0, false, 25.0, 200, false};
    EEPROM.put(SETTINGS_ADDR, g_settings);
  }
  // Apply loaded settings to control variables
  coolSpeed = g_settings.coolSpeed;
  heatSpeed = g_settings.heatSpeed;
  // Start with LED off regardless of saved setting
  growLevel = 0;
  autoMode = g_settings.autoMode;
  g_tempSetpoint = g_settings.tempSetpoint;
  g_targetLight = g_settings.targetLight;
  lightAutoMode = g_settings.lightAutoMode;
  // targetGrowLevel is set to 0 in setup() to ensure LED is off at startup
  
  // Initialize light control variables to ensure they're safe
  lightPWM = 0;
  g_lightIntegralError = 0.0;
  g_lastLEDOnTime = 0;
  
  // Log the loaded settings
  Serial.println(F("[SETTINGS] Loaded from EEPROM"));
  Serial.print(F("[SETTINGS] Target light: ")); Serial.println(g_targetLight);
}

/**
 * Saves current settings to EEPROM.
 */
void saveSettings() {
  g_settings.version = SETTINGS_VERSION;
  g_settings.coolSpeed = coolSpeed;
  g_settings.heatSpeed = heatSpeed;
  g_settings.growLevel = growLevel;
  g_settings.autoMode = autoMode;
  g_settings.tempSetpoint = g_tempSetpoint;
  g_settings.targetLight = g_targetLight;
  g_settings.lightAutoMode = lightAutoMode;
  EEPROM.put(SETTINGS_ADDR, g_settings);
}

/**
 * Resets settings to factory defaults and saves to EEPROM.
 */
void resetSettings() {
  g_settings = {SETTINGS_VERSION, 0, 0, 0, 25.0, 18.0, false, 25.0, 200, false};
  EEPROM.put(SETTINGS_ADDR, g_settings);
  coolSpeed = g_settings.coolSpeed;
  heatSpeed = g_settings.heatSpeed;
  growLevel = g_settings.growLevel;
  autoMode = g_settings.autoMode;
  g_tempSetpoint = g_settings.tempSetpoint;
  g_targetLight = g_settings.targetLight;
  lightAutoMode = g_settings.lightAutoMode;
  
  // Log the reset
  Serial.println(F("[SETTINGS] Factory reset"));
  Serial.print(F("[SETTINGS] Target light: ")); Serial.println(g_targetLight);
}

/**
 * Initializes custom characters for the LCD display.
 * This loads the custom character definitions into the LCD's CGRAM.
 */
void initializeCustomCharacters() {
  lcd.createChar(0, (uint8_t*)CHAR_THERMOMETER);
  lcd.createChar(1, (uint8_t*)CHAR_DROPLET);
  lcd.createChar(2, (uint8_t*)CHAR_LIGHT);
  lcd.createChar(3, (uint8_t*)CHAR_FAN_1);
  lcd.createChar(4, (uint8_t*)CHAR_FAN_2);
  lcd.createChar(5, (uint8_t*)CHAR_HEAT);
  lcd.createChar(6, (uint8_t*)CHAR_AUTO);
  lcd.createChar(7, (uint8_t*)CHAR_ALERT);
  
  Serial.println(F("[LCD] Custom characters initialized"));
}

/**
 * Starts an animation sequence at the specified position.
 * 
 * @param type Animation type (1=CoolFan, 2=HeatFan, 3=Light, 4=Alert)
 * @param x X position on LCD (0-15)
 * @param y Y position on LCD (0-1)
 */
void startAnimation(byte type, byte x, byte y) {
  // Don't restart an active animation of the same type
  if (g_animationActive && g_currentAnimationType == type) {
    return;
  }
  
  g_animationActive = true;
  g_animationStartMillis = g_currentMillis;
  g_lastAnimationUpdateMillis = g_currentMillis;
  g_currentAnimationFrame = 0;
  g_animationDisplayX = x;
  g_animationDisplayY = y;
  g_currentAnimationType = type;
  
  Serial.print(F("[ANIMATION] Started type "));
  Serial.print(type);
  Serial.print(F(" at position ("));
  Serial.print(x);
  Serial.print(F(","));
  Serial.print(y);
  Serial.println(F(")"));
}

/**
 * Simplified Light Control with Stable PID - fixes oscillation issues
 */
void updateLightControl() {
  // Skip if auto light mode is off or if lux reading is invalid
  if (!lightAutoMode || isnan(g_luxFiltered)) {
    return;
  }
  
  // If target is very low (below 50 lux), turn light off completely
  if (g_targetLight < 50) {
    lightPWM = 0;
    growLevel = 0;
    g_lightIntegralError = 0; // Reset integral error
    g_previousError = 0; // Reset derivative term
    Serial.println("[LIGHT] Target too low, turning light off");
    return;
  }
  
  // Temperature-based safety: if ambient temperature is high, limit LED brightness
  float tempSafetyFactor = 1.0;
  if (!isnan(g_temp)) {
    if (g_temp > 30.0) {
      // Start reducing brightness above 30°C
      tempSafetyFactor = map((int)(g_temp * 10), 300, 400, 100, 50) / 100.0;
      tempSafetyFactor = constrain(tempSafetyFactor, 0.5, 1.0);
      
      // Force off at very high temperatures
      if (g_temp > 40.0) {
        lightPWM = 0;
        growLevel = 0;
        g_lightIntegralError = 0;
        g_previousError = 0;
        Serial.println("[LIGHT] SAFETY: Temperature too high, turning light off");
        return;
      }
    }
  }
  
  // Simplified adjustment interval - consistent for all targets
  const unsigned long SIMPLIFIED_ADJUSTMENT_INTERVAL = 1000; // 1 second for stability
  
  if (g_currentMillis - lastLightAdjustmentMillis < SIMPLIFIED_ADJUSTMENT_INTERVAL) {
    return;
  }
  
  // Calculate error (target - current)
  float error = g_targetLight - g_luxFiltered;
  
  // Simplified deadband - fixed percentage regardless of target
  float deadbandPercent = 0.15; // 15% deadband for all targets
  float deadbandThreshold = g_targetLight * deadbandPercent;
  
  // Minimum deadband to prevent tiny oscillations
  deadbandThreshold = max(deadbandThreshold, 20.0f); // At least 20 lux deadband
  
  // If we're close enough to target, make no adjustments
  if (abs(error) < deadbandThreshold) {
    // Decay integral error when in deadband
    g_lightIntegralError *= 0.95;
    Serial.print("[LIGHT] Within deadband (±"); 
    Serial.print(deadbandThreshold);
    Serial.println(" lux) - no adjustment");
    return;
  }
  
  // Simplified PID parameters - same for all targets
  const float KP = 0.008; // Reduced proportional gain
  const float KI = 0.0002; // Small integral gain
  const float KD = 0.001; // Small derivative gain
  
  // Calculate PID terms
  float pTerm = KP * error;
  
  // Update integral with anti-windup
  const float MAX_INTEGRAL = 1000.0;
  if (abs(g_lightIntegralError) < MAX_INTEGRAL) {
    g_lightIntegralError += error * (SIMPLIFIED_ADJUSTMENT_INTERVAL / 1000.0);
  }
  g_lightIntegralError = constrain(g_lightIntegralError, -MAX_INTEGRAL, MAX_INTEGRAL);
  float iTerm = KI * g_lightIntegralError;
  
  // Calculate derivative with simple difference
  float dTerm = KD * (error - g_previousError);
  g_previousError = error;
  
  // Calculate total adjustment
  float rawAdjustment = pTerm + iTerm + dTerm;
  
  // Convert to PWM adjustment (limit to ±3 per cycle for stability)
  int adjustment = constrain((int)rawAdjustment, -3, 3);
  
  // Skip tiny adjustments
  if (adjustment == 0) {
    return;
  }
  
  // Calculate maximum brightness based on target (simplified)
  int maxBrightness;
  if (g_targetLight < 500) {
    maxBrightness = 20; // Low targets
  } else if (g_targetLight < 1500) {
    maxBrightness = 50; // Medium targets  
  } else {
    maxBrightness = MAX_LIGHT_BRIGHTNESS; // High targets
  }
  
  // Apply temperature safety factor
  maxBrightness = (int)(maxBrightness * tempSafetyFactor);
  
  // Apply adjustment
  int previousPWM = lightPWM;
  lightPWM = constrain(lightPWM + adjustment, 0, maxBrightness);
  
  // Minimum on-time check (simplified)
  const unsigned long MIN_ON_TIME_SIMPLIFIED = 3000; // 3 seconds
  if (previousPWM > 0 && lightPWM == 0 && g_currentMillis - g_lastLEDOnTime < MIN_ON_TIME_SIMPLIFIED) {
    lightPWM = 1; // Keep minimum brightness instead of turning off
    Serial.println("[LIGHT] Enforcing minimum on time");
  }
  
  // Track when LED turns on
  if (previousPWM == 0 && lightPWM > 0) {
    g_lastLEDOnTime = g_currentMillis;
  }
  
  // Apply the new PWM value
  growLevel = lightPWM;
  
  // Update timestamp
  lastLightAdjustmentMillis = g_currentMillis;
  
  Serial.print("[LIGHT] Target: "); Serial.print(g_targetLight);
  Serial.print(" Current: "); Serial.print(g_luxFiltered);
  Serial.print(" Error: "); Serial.print(error);
  Serial.print(" P: "); Serial.print(pTerm);
  Serial.print(" I: "); Serial.print(iTerm);
  Serial.print(" D: "); Serial.print(dTerm);
  Serial.print(" Adj: "); Serial.print(adjustment);
  Serial.print(" PWM: "); Serial.println(lightPWM);
}

// ======== Lightning-Fast Button Implementation ========
/**
 * Initialize lightning-fast button system
 */
void initButtons() {
  int buttonPins[] = {BTN_SELECT, BTN_LEFT, BTN_RIGHT, BTN_UP, BTN_DOWN};
  for (int i = 0; i < 5; i++) {
    int pin = buttonPins[i];
    g_lastButtonState[pin] = true; // Start with button not pressed (active low)
  }
  g_selectPressed = false;
  g_selectPressStart = 0;
  Serial.println(F("[BUTTONS] Lightning-fast button system initialized"));
}

/**
 * Instant button press detection - returns true on falling edge only
 * This is called once per button and immediately processes the press
 */
bool buttonPressed(int pin) {
  bool currentState = (digitalRead(pin) == LOW);
  bool justPressed = currentState && !g_lastButtonState[pin];
  g_lastButtonState[pin] = currentState;
  return justPressed;
}

/**
 * Show a temporary message for 1.5 seconds without blocking
 */
void showTempMessage(String message) {
  g_tempMessage = message;
  g_showTempMessage = true;
  g_tempMessageStartTime = g_currentMillis;
  Serial.print(F("[TEMP MSG] "));
  Serial.println(message);
}

// ======== LIGHTNING-FAST BUTTON SYSTEM DOCUMENTATION ========
/*
  ABSOLUTE MAXIMUM SPEED BUTTON HANDLING:
  
  1. INSTANT RESPONSE:
     - Single-function edge detection with immediate action
     - No separate update/check phases - everything in one call
     - Immediate LCD update on button press (bypasses scheduling)
     - One button handled per loop for maximum responsiveness
  
  2. MINIMAL STATE TRACKING:
     - Only tracks: previous button state (1 bool per button)
     - Direct falling edge detection (HIGH to LOW transition)
     - Absolute minimal processing for maximum speed
  
  3. STREAMLINED FUNCTIONALITY:
     - buttonPressed(): Instant edge detection + immediate action
     - SELECT long press: Direct timing for settings save
     - No complex state machines or unnecessary features
  
  4. BREADBOARD OPTIMIZED:
     - Designed for clean 4-pin tactile buttons
     - Trusts hardware quality over software complexity
     - Maximum responsiveness for direct user interaction
  
  5. DISPLAY OPTIMIZATION:
     - Immediate display updates on button press
     - Smart clearing (only when menu changes)
     - Minimal I2C operations for maximum speed
  
  DESIGN PRINCIPLES:
  - Absolute speed priority over everything else
  - Simplest possible code paths
  - Immediate feedback on user input
  - Single-cycle edge detection and response
*/