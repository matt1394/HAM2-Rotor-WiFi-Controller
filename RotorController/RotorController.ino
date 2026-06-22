/*
 * CDE HAM-2 Antenna Rotor WiFi Controller
 *
 * Full-featured WiFi remote control for CDE HAM-2 antenna rotor
 * with safety-first design principles for mechanical control.
 *
 * Hardware: ESP32 Dev Module with integrated 4-channel relay module
 *
 * SAFETY CRITICAL: This firmware controls mechanical equipment.
 * All timing sequences must be followed exactly to prevent damage.
 *
 * Author: Amateur Radio Project
 * License: MIT
 */

#include <WiFi.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <EEPROM.h>

// ============================================================================
// FIRMWARE VERSION
// ============================================================================
#define FIRMWARE_VERSION "1.0.0"

// ============================================================================
// HARDWARE CONFIGURATION - ACTIVE LOW RELAY LOGIC
// ============================================================================

// Relay pin assignments (validated GPIO mapping)
#define RELAY_BRAKE   32    // Relay 1: Brake Release
#define RELAY_CW      33    // Relay 2: Clockwise Rotation
#define RELAY_CCW     25    // Relay 3: Counter-Clockwise Rotation
#define RELAY_SPARE   26    // Relay 4: Unused (available for expansion)

// WiFi reset button (GPIO 0 = built-in BOOT button on most ESP32 dev modules)
// Hold for WIFI_RESET_HOLD_MS to wipe saved credentials and reboot into config portal
#define WIFI_RESET_BUTTON     0
#define WIFI_RESET_HOLD_MS    3000

// ADC for position feedback
#define ADC_POSITION  34    // ADC1_CH6 (input only, no pullup - ideal for ADC)

// Relay logic configuration
#define RELAY_ACTIVE_LOW true

// All relay pins for initialization
const int relayPins[] = {RELAY_BRAKE, RELAY_CW, RELAY_CCW, RELAY_SPARE};
const int relayCount = sizeof(relayPins) / sizeof(relayPins[0]);

// ============================================================================
// SAFETY TIMING CONSTANTS - DO NOT MODIFY WITHOUT CAREFUL CONSIDERATION
// ============================================================================

// Brake release delay - brake must fully disengage before rotation
#define BRAKE_RELEASE_DELAY_MS    1000    // 1 second minimum

// Post-rotation delay - rotor must stop before brake re-engages
#define POST_ROTATION_DELAY_MS    1500    // 1.5 seconds

// Maximum continuous rotation time (safety timeout)
#define MAX_ROTATION_TIME_MS      60000   // 60 seconds max

// Watchdog timeout - detect hung states
#define WATCHDOG_TIMEOUT_MS       5000    // 5 seconds

// Position reading interval
#define POSITION_READ_INTERVAL_MS 100     // 10 Hz sampling

// Web interface update interval
#define WEB_UPDATE_INTERVAL_MS    250     // 4 Hz updates

// ============================================================================
// POSITION FEEDBACK CONFIGURATION
// ============================================================================

// ADC characteristics
#define ADC_RESOLUTION      12          // ESP32 has 12-bit ADC
#define ADC_MAX_VALUE       4095        // 2^12 - 1
#define ADC_SAMPLES         10          // Number of samples to average
#define ADC_SAMPLE_DELAY_MS 10          // Delay between samples

// Default calibration values (will be overwritten from EEPROM)
#define DEFAULT_ADC_MIN     100         // ADC reading at 0 degrees
#define DEFAULT_ADC_MAX     2800        // ADC reading at 360 degrees

// ============================================================================
// NETWORK CONFIGURATION
// ============================================================================

#define MDNS_HOSTNAME       "rotor-control"
#define AP_NAME             "Rotor-Controller-Setup"
#define WEB_SERVER_PORT     80

// ============================================================================
// EEPROM CONFIGURATION
// ============================================================================

#define EEPROM_SIZE         64
#define EEPROM_MAGIC        0xAA55      // Magic number to validate EEPROM data
#define EEPROM_ADDR_MAGIC   0
#define EEPROM_ADDR_ADC_MIN 2
#define EEPROM_ADDR_ADC_MAX 4

// ============================================================================
// STATE MACHINE
// ============================================================================

enum RotorState {
  STATE_IDLE,             // No operation, brake engaged
  STATE_BRAKE_RELEASING,  // Waiting for brake to disengage
  STATE_ROTATING_CW,      // Active clockwise rotation
  STATE_ROTATING_CCW,     // Active counter-clockwise rotation
  STATE_STOPPING,         // Rotation stopped, waiting before brake engages
  STATE_EMERGENCY_STOP    // Immediate halt, safety mode
};

const char* stateNames[] = {
  "IDLE",
  "BRAKE_RELEASING",
  "ROTATING_CW",
  "ROTATING_CCW",
  "STOPPING",
  "EMERGENCY_STOP"
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// State machine
volatile RotorState currentState = STATE_IDLE;
volatile RotorState pendingDirection = STATE_IDLE;  // Direction to rotate after brake release
unsigned long stateStartTime = 0;
unsigned long rotationStartTime = 0;

// Position tracking
int currentPositionADC = 0;
float currentPositionDegrees = 0.0;
int adcMin = DEFAULT_ADC_MIN;
int adcMax = DEFAULT_ADC_MAX;

// Automatic rotation
bool autoRotateEnabled = false;
float targetHeading = 0.0;
float headingTolerance = 2.0;  // Degrees tolerance for auto-stop

// Timing
unsigned long lastPositionRead = 0;
unsigned long lastWatchdogReset = 0;

// Web server
WebServer server(WEB_SERVER_PORT);

// Status tracking
String lastAction = "System startup";
unsigned long lastActionTime = 0;

// WiFi reset button
unsigned long wifiResetButtonPressTime = 0;
bool wifiResetButtonActive = false;

// ============================================================================
// RELAY CONTROL FUNCTIONS (ACTIVE LOW LOGIC)
// ============================================================================

inline void relayOff(int pin) {
  digitalWrite(pin, RELAY_ACTIVE_LOW ? HIGH : LOW);
}

inline void relayOn(int pin) {
  digitalWrite(pin, RELAY_ACTIVE_LOW ? LOW : HIGH);
}

// Turn all relays off - SAFETY FUNCTION
void allRelaysOff() {
  relayOff(RELAY_BRAKE);
  relayOff(RELAY_CW);
  relayOff(RELAY_CCW);
  relayOff(RELAY_SPARE);
}

// Initialize all relays to OFF state before setting as outputs
// This prevents relay glitches during startup
void initializeRelays() {
  Serial.println("[INIT] Initializing relays to OFF state...");

  for (int i = 0; i < relayCount; i++) {
    int pin = relayPins[i];
    relayOff(pin);              // Set output latch to OFF
    pinMode(pin, OUTPUT);       // Start driving the pin
    relayOff(pin);              // Enforce OFF again
    Serial.printf("[INIT] Relay on GPIO %d: OFF\n", pin);
  }

  Serial.println("[INIT] All relays initialized OFF");
}

// ============================================================================
// POSITION READING FUNCTIONS
// ============================================================================

// Read ADC with averaging for noise reduction
int readPositionADC() {
  long sum = 0;

  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(ADC_POSITION);
    if (i < ADC_SAMPLES - 1) {
      delay(ADC_SAMPLE_DELAY_MS);
    }
  }

  return sum / ADC_SAMPLES;
}

// Convert ADC reading to degrees (0-360)
// Note: CDE HAM-2 indicator starts at South (180°) at 0V and sweeps
// CW through 360° back to South (180°) at ~12.5V
float adcToDegrees(int adcValue) {
  // Constrain to calibration range
  int constrained = constrain(adcValue, adcMin, adcMax);

  // Linear mapping with 180° offset (meter starts at South)
  // 0V (adcMin) = 180°, full scale (adcMax) = 180° + 360° = 540° = 180°
  float degrees = 180.0 + (float)(constrained - adcMin) / (float)(adcMax - adcMin) * 360.0;

  // Wrap to 0-360 range
  while (degrees >= 360.0) degrees -= 360.0;
  while (degrees < 0) degrees += 360.0;

  return degrees;
}

// Update position reading
void updatePosition() {
  currentPositionADC = readPositionADC();
  currentPositionDegrees = adcToDegrees(currentPositionADC);
}

// ============================================================================
// CALIBRATION FUNCTIONS (EEPROM)
// ============================================================================

void loadCalibration() {
  EEPROM.begin(EEPROM_SIZE);

  uint16_t magic = EEPROM.read(EEPROM_ADDR_MAGIC) | (EEPROM.read(EEPROM_ADDR_MAGIC + 1) << 8);

  if (magic == EEPROM_MAGIC) {
    adcMin = EEPROM.read(EEPROM_ADDR_ADC_MIN) | (EEPROM.read(EEPROM_ADDR_ADC_MIN + 1) << 8);
    adcMax = EEPROM.read(EEPROM_ADDR_ADC_MAX) | (EEPROM.read(EEPROM_ADDR_ADC_MAX + 1) << 8);
    Serial.printf("[CAL] Loaded calibration: min=%d, max=%d\n", adcMin, adcMax);
  } else {
    Serial.println("[CAL] No valid calibration found, using defaults");
    adcMin = DEFAULT_ADC_MIN;
    adcMax = DEFAULT_ADC_MAX;
  }

  EEPROM.end();
}

void saveCalibration() {
  EEPROM.begin(EEPROM_SIZE);

  // Write magic number
  EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC & 0xFF);
  EEPROM.write(EEPROM_ADDR_MAGIC + 1, (EEPROM_MAGIC >> 8) & 0xFF);

  // Write calibration values
  EEPROM.write(EEPROM_ADDR_ADC_MIN, adcMin & 0xFF);
  EEPROM.write(EEPROM_ADDR_ADC_MIN + 1, (adcMin >> 8) & 0xFF);
  EEPROM.write(EEPROM_ADDR_ADC_MAX, adcMax & 0xFF);
  EEPROM.write(EEPROM_ADDR_ADC_MAX + 1, (adcMax >> 8) & 0xFF);

  EEPROM.commit();
  EEPROM.end();

  Serial.printf("[CAL] Saved calibration: min=%d, max=%d\n", adcMin, adcMax);
}

// ============================================================================
// STATE MACHINE FUNCTIONS
// ============================================================================

void logStateTransition(RotorState from, RotorState to, const char* reason) {
  Serial.printf("[STATE] %s -> %s (%s)\n", stateNames[from], stateNames[to], reason);
  lastAction = String(stateNames[to]) + ": " + String(reason);
  lastActionTime = millis();
}

// Enter a new state
void enterState(RotorState newState, const char* reason) {
  if (newState == currentState) return;

  RotorState oldState = currentState;
  currentState = newState;
  stateStartTime = millis();

  logStateTransition(oldState, newState, reason);

  // Handle state entry actions
  switch (newState) {
    case STATE_IDLE:
      allRelaysOff();
      autoRotateEnabled = false;
      break;

    case STATE_BRAKE_RELEASING:
      // Turn on brake release, keep rotation off
      relayOff(RELAY_CW);
      relayOff(RELAY_CCW);
      relayOn(RELAY_BRAKE);
      break;

    case STATE_ROTATING_CW:
      relayOn(RELAY_BRAKE);  // Keep brake released
      relayOn(RELAY_CW);
      relayOff(RELAY_CCW);
      rotationStartTime = millis();
      break;

    case STATE_ROTATING_CCW:
      relayOn(RELAY_BRAKE);  // Keep brake released
      relayOff(RELAY_CW);
      relayOn(RELAY_CCW);
      rotationStartTime = millis();
      break;

    case STATE_STOPPING:
      relayOff(RELAY_CW);
      relayOff(RELAY_CCW);
      relayOn(RELAY_BRAKE);  // Keep brake released during deceleration
      break;

    case STATE_EMERGENCY_STOP:
      allRelaysOff();
      autoRotateEnabled = false;
      Serial.println("[SAFETY] EMERGENCY STOP ACTIVATED");
      break;
  }
}

// Calculate time in current state
unsigned long timeInState() {
  return millis() - stateStartTime;
}

// Calculate time rotating
unsigned long timeRotating() {
  return millis() - rotationStartTime;
}

// ============================================================================
// ROTATION CONTROL FUNCTIONS
// ============================================================================

// Start rotation in specified direction (handles brake release sequence)
bool startRotation(RotorState direction) {
  if (direction != STATE_ROTATING_CW && direction != STATE_ROTATING_CCW) {
    Serial.println("[ERROR] Invalid rotation direction");
    return false;
  }

  if (currentState == STATE_EMERGENCY_STOP) {
    Serial.println("[SAFETY] Cannot rotate - emergency stop active");
    return false;
  }

  if (currentState == STATE_ROTATING_CW || currentState == STATE_ROTATING_CCW) {
    Serial.println("[WARN] Already rotating");
    return false;
  }

  if (currentState == STATE_BRAKE_RELEASING || currentState == STATE_STOPPING) {
    Serial.println("[WARN] Operation in progress");
    return false;
  }

  // Store pending direction and start brake release sequence
  pendingDirection = direction;
  enterState(STATE_BRAKE_RELEASING, direction == STATE_ROTATING_CW ? "CW requested" : "CCW requested");

  return true;
}

// Stop rotation (handles brake engagement sequence)
void stopRotation() {
  if (currentState == STATE_ROTATING_CW || currentState == STATE_ROTATING_CCW) {
    enterState(STATE_STOPPING, "Stop requested");
  } else if (currentState == STATE_BRAKE_RELEASING) {
    // Abort before rotation started
    enterState(STATE_STOPPING, "Aborted during brake release");
  }
}

// Emergency stop - immediate halt
void emergencyStop() {
  enterState(STATE_EMERGENCY_STOP, "Emergency stop");
}

// Clear emergency stop and return to idle
void clearEmergencyStop() {
  if (currentState == STATE_EMERGENCY_STOP) {
    enterState(STATE_IDLE, "Emergency cleared");
  }
}

// ============================================================================
// AUTOMATIC ROTATION
// ============================================================================

// Calculate shortest rotation direction to target
// Returns STATE_ROTATING_CW or STATE_ROTATING_CCW
RotorState calculateDirection(float current, float target) {
  float diff = target - current;

  // Normalize to -180 to +180
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;

  return (diff > 0) ? STATE_ROTATING_CW : STATE_ROTATING_CCW;
}

// Check if we've reached the target heading
bool isAtTarget(float current, float target, float tolerance) {
  float diff = abs(target - current);

  // Handle wraparound at 0/360
  if (diff > 180) diff = 360 - diff;

  return diff <= tolerance;
}

// Start automatic rotation to heading
bool goToHeading(float heading) {
  if (heading < 0 || heading >= 360) {
    Serial.printf("[ERROR] Invalid heading: %.1f\n", heading);
    return false;
  }

  if (currentState != STATE_IDLE) {
    Serial.println("[WARN] Cannot start auto-rotate - not idle");
    return false;
  }

  // Check if already at target
  if (isAtTarget(currentPositionDegrees, heading, headingTolerance)) {
    Serial.printf("[AUTO] Already at target heading %.1f\n", heading);
    lastAction = "Already at target";
    lastActionTime = millis();
    return true;
  }

  targetHeading = heading;
  autoRotateEnabled = true;

  RotorState direction = calculateDirection(currentPositionDegrees, heading);
  Serial.printf("[AUTO] Rotating to %.1f degrees (%s)\n", heading,
                direction == STATE_ROTATING_CW ? "CW" : "CCW");

  return startRotation(direction);
}

// ============================================================================
// STATE MACHINE UPDATE (called in loop)
// ============================================================================

void updateStateMachine() {
  unsigned long now = millis();

  switch (currentState) {
    case STATE_IDLE:
      // Nothing to do
      break;

    case STATE_BRAKE_RELEASING:
      // Wait for brake release delay, then start rotation
      if (timeInState() >= BRAKE_RELEASE_DELAY_MS) {
        if (pendingDirection == STATE_ROTATING_CW) {
          enterState(STATE_ROTATING_CW, "Brake released");
        } else if (pendingDirection == STATE_ROTATING_CCW) {
          enterState(STATE_ROTATING_CCW, "Brake released");
        } else {
          // No direction pending, go to stopping
          enterState(STATE_STOPPING, "No direction pending");
        }
      }
      break;

    case STATE_ROTATING_CW:
    case STATE_ROTATING_CCW:
      // Check safety timeout
      if (timeRotating() >= MAX_ROTATION_TIME_MS) {
        Serial.println("[SAFETY] Maximum rotation time exceeded");
        stopRotation();
        break;
      }

      // Check auto-rotate target
      if (autoRotateEnabled) {
        if (isAtTarget(currentPositionDegrees, targetHeading, headingTolerance)) {
          Serial.printf("[AUTO] Target reached: %.1f degrees\n", currentPositionDegrees);
          autoRotateEnabled = false;
          stopRotation();
        }
      }
      break;

    case STATE_STOPPING:
      // Wait for post-rotation delay, then engage brake
      if (timeInState() >= POST_ROTATION_DELAY_MS) {
        enterState(STATE_IDLE, "Stop complete");
      }
      break;

    case STATE_EMERGENCY_STOP:
      // Stay in emergency stop until manually cleared
      break;
  }

  // Watchdog - reset if state machine is responsive
  lastWatchdogReset = now;
}

// ============================================================================
// WEB SERVER HANDLERS
// ============================================================================

// Serve main page
void handleRoot() {
  server.send(200, "text/html", getWebPage());
}

// API: Get current status (JSON)
void handleStatus() {
  String json = "{";
  json += "\"state\":\"" + String(stateNames[currentState]) + "\",";
  json += "\"position_adc\":" + String(currentPositionADC) + ",";
  json += "\"position_degrees\":" + String(currentPositionDegrees, 1) + ",";
  json += "\"auto_rotate\":" + String(autoRotateEnabled ? "true" : "false") + ",";
  json += "\"target_heading\":" + String(targetHeading, 1) + ",";
  json += "\"adc_min\":" + String(adcMin) + ",";
  json += "\"adc_max\":" + String(adcMax) + ",";
  json += "\"last_action\":\"" + lastAction + "\",";
  json += "\"uptime\":" + String(millis() / 1000) + ",";
  json += "\"wifi_rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"firmware\":\"" + String(FIRMWARE_VERSION) + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

// API: Start CW rotation
void handleCW() {
  if (startRotation(STATE_ROTATING_CW)) {
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Cannot start rotation");
  }
}

// API: Start CCW rotation
void handleCCW() {
  if (startRotation(STATE_ROTATING_CCW)) {
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Cannot start rotation");
  }
}

// API: Stop rotation
void handleStop() {
  stopRotation();
  server.send(200, "text/plain", "OK");
}

// API: Emergency stop
void handleEmergencyStop() {
  emergencyStop();
  server.send(200, "text/plain", "EMERGENCY STOP ACTIVATED");
}

// API: Clear emergency stop
void handleClearEmergency() {
  clearEmergencyStop();
  server.send(200, "text/plain", "OK");
}

// API: Go to heading
void handleGoTo() {
  if (!server.hasArg("heading")) {
    server.send(400, "text/plain", "Missing heading parameter");
    return;
  }

  float heading = server.arg("heading").toFloat();

  if (goToHeading(heading)) {
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Cannot go to heading");
  }
}

// API: Set calibration min (current ADC = 0 degrees)
void handleCalMin() {
  adcMin = currentPositionADC;
  saveCalibration();
  server.send(200, "text/plain", "Calibration min set to " + String(adcMin));
}

// API: Set calibration max (current ADC = 360 degrees)
void handleCalMax() {
  adcMax = currentPositionADC;
  saveCalibration();
  server.send(200, "text/plain", "Calibration max set to " + String(adcMax));
}

// API: Reset calibration to defaults
void handleCalReset() {
  adcMin = DEFAULT_ADC_MIN;
  adcMax = DEFAULT_ADC_MAX;
  saveCalibration();
  server.send(200, "text/plain", "Calibration reset to defaults");
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/cw", handleCW);
  server.on("/ccw", handleCCW);
  server.on("/stop", handleStop);
  server.on("/emergency", handleEmergencyStop);
  server.on("/clear", handleClearEmergency);
  server.on("/goto", handleGoTo);
  server.on("/cal/min", handleCalMin);
  server.on("/cal/max", handleCalMax);
  server.on("/cal/reset", handleCalReset);

  server.begin();
  Serial.printf("[WEB] Server started on port %d\n", WEB_SERVER_PORT);
}

// ============================================================================
// WEB PAGE (embedded HTML/CSS/JS)
// ============================================================================

String getWebPage() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Rotor Control</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
      background: #1a1a2e;
      color: #eee;
      min-height: 100vh;
      padding: 20px;
    }
    .container { max-width: 600px; margin: 0 auto; }
    h1 { text-align: center; margin-bottom: 20px; color: #00d4ff; }

    /* Position Display */
    .position-display {
      background: #16213e;
      border-radius: 20px;
      padding: 30px;
      text-align: center;
      margin-bottom: 20px;
      border: 2px solid #0f3460;
    }
    .heading {
      font-size: 72px;
      font-weight: bold;
      color: #00d4ff;
      text-shadow: 0 0 20px rgba(0,212,255,0.5);
    }
    .heading-unit { font-size: 36px; color: #888; }
    .adc-value { font-size: 14px; color: #666; margin-top: 10px; }

    /* Compass */
    .compass {
      width: 200px;
      height: 200px;
      margin: 20px auto;
      position: relative;
      border-radius: 50%;
      background: conic-gradient(from 0deg, #0f3460, #16213e, #0f3460);
      border: 3px solid #00d4ff;
    }
    .compass-needle {
      position: absolute;
      width: 4px;
      height: 80px;
      background: linear-gradient(to top, #ff4444, #ff4444 50%, #ffffff 50%);
      left: 50%;
      bottom: 50%;
      transform-origin: bottom center;
      transform: translateX(-50%) rotate(0deg);
      border-radius: 2px;
      transition: transform 0.3s ease;
    }
    .compass-center {
      position: absolute;
      width: 16px;
      height: 16px;
      background: #00d4ff;
      border-radius: 50%;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
    }
    .compass-label {
      position: absolute;
      font-size: 14px;
      font-weight: bold;
      color: #888;
    }
    .compass-label.n { top: 10px; left: 50%; transform: translateX(-50%); color: #00d4ff; }
    .compass-label.s { bottom: 10px; left: 50%; transform: translateX(-50%); }
    .compass-label.e { right: 10px; top: 50%; transform: translateY(-50%); }
    .compass-label.w { left: 10px; top: 50%; transform: translateY(-50%); }

    /* Status */
    .status {
      background: #16213e;
      border-radius: 10px;
      padding: 15px;
      margin-bottom: 20px;
      border-left: 4px solid #00d4ff;
    }
    .status-row { display: flex; justify-content: space-between; margin: 5px 0; }
    .status-label { color: #888; }
    .status-value { font-weight: bold; }
    .status-value.idle { color: #4ade80; }
    .status-value.rotating { color: #fbbf24; }
    .status-value.emergency { color: #ef4444; }

    /* Controls */
    .controls { display: grid; gap: 15px; margin-bottom: 20px; }
    .control-row { display: flex; gap: 15px; }

    button {
      flex: 1;
      padding: 20px;
      font-size: 18px;
      font-weight: bold;
      border: none;
      border-radius: 10px;
      cursor: pointer;
      transition: all 0.2s;
      text-transform: uppercase;
    }
    button:active { transform: scale(0.95); }
    button:disabled { opacity: 0.5; cursor: not-allowed; }

    .btn-cw { background: #3b82f6; color: white; }
    .btn-cw:hover:not(:disabled) { background: #2563eb; }
    .btn-ccw { background: #8b5cf6; color: white; }
    .btn-ccw:hover:not(:disabled) { background: #7c3aed; }
    .btn-stop { background: #f59e0b; color: white; }
    .btn-stop:hover:not(:disabled) { background: #d97706; }
    .btn-emergency { background: #ef4444; color: white; font-size: 24px; }
    .btn-emergency:hover:not(:disabled) { background: #dc2626; }
    .btn-clear { background: #10b981; color: white; }
    .btn-clear:hover:not(:disabled) { background: #059669; }

    /* Go To Heading */
    .goto-section {
      background: #16213e;
      border-radius: 10px;
      padding: 20px;
      margin-bottom: 20px;
    }
    .goto-section h3 { margin-bottom: 15px; color: #00d4ff; }
    .goto-input {
      display: flex;
      gap: 10px;
    }
    .goto-input input {
      flex: 1;
      padding: 15px;
      font-size: 18px;
      border: 2px solid #0f3460;
      border-radius: 10px;
      background: #1a1a2e;
      color: white;
      text-align: center;
    }
    .goto-input input:focus { outline: none; border-color: #00d4ff; }
    .btn-goto { background: #06b6d4; color: white; flex: 0 0 120px; }
    .btn-goto:hover:not(:disabled) { background: #0891b2; }

    /* Calibration */
    .calibration {
      background: #16213e;
      border-radius: 10px;
      padding: 20px;
      margin-bottom: 20px;
    }
    .calibration h3 { margin-bottom: 15px; color: #888; }
    .cal-values { font-size: 14px; color: #666; margin-bottom: 15px; }
    .cal-buttons { display: flex; gap: 10px; }
    .btn-cal { background: #374151; color: #ccc; padding: 10px; font-size: 14px; }
    .btn-cal:hover:not(:disabled) { background: #4b5563; }

    /* Info */
    .info {
      text-align: center;
      font-size: 12px;
      color: #666;
    }

    /* Auto-rotate indicator */
    .auto-indicator {
      display: none;
      background: #06b6d4;
      color: white;
      padding: 10px;
      border-radius: 10px;
      text-align: center;
      margin-bottom: 15px;
      animation: pulse 2s infinite;
    }
    .auto-indicator.active { display: block; }
    @keyframes pulse {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.7; }
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>Rotor Control</h1>

    <div class="position-display">
      <div class="compass">
        <div class="compass-needle" id="needle"></div>
        <div class="compass-center"></div>
        <span class="compass-label n">N</span>
        <span class="compass-label s">S</span>
        <span class="compass-label e">E</span>
        <span class="compass-label w">W</span>
      </div>
      <div class="heading"><span id="heading">---</span><span class="heading-unit">&deg;</span></div>
      <div class="adc-value">ADC: <span id="adc">----</span></div>
    </div>

    <div class="auto-indicator" id="autoIndicator">
      Auto-rotating to <span id="targetDisplay">---</span>&deg;
    </div>

    <div class="status">
      <div class="status-row">
        <span class="status-label">Status:</span>
        <span class="status-value" id="state">---</span>
      </div>
      <div class="status-row">
        <span class="status-label">Last Action:</span>
        <span class="status-value" id="lastAction">---</span>
      </div>
      <div class="status-row">
        <span class="status-label">WiFi Signal:</span>
        <span class="status-value" id="rssi">---</span>
      </div>
    </div>

    <div class="controls">
      <div class="control-row">
        <button class="btn-ccw" onmousedown="startCCW()" onmouseup="stop()" onmouseleave="stop()" ontouchstart="startCCW()" ontouchend="stop()">&#x21BA; CCW</button>
        <button class="btn-stop" onclick="stop()">STOP</button>
        <button class="btn-cw" onmousedown="startCW()" onmouseup="stop()" onmouseleave="stop()" ontouchstart="startCW()" ontouchend="stop()">CW &#x21BB;</button>
      </div>
      <button class="btn-emergency" onclick="emergency()">EMERGENCY STOP</button>
      <button class="btn-clear" id="clearBtn" onclick="clearEmergency()" style="display:none;">Clear Emergency</button>
    </div>

    <div class="goto-section">
      <h3>Go To Heading</h3>
      <div class="goto-input">
        <input type="number" id="targetHeading" min="0" max="359" placeholder="0-359">
        <button class="btn-goto" onclick="goToHeading()">GO</button>
      </div>
    </div>

    <div class="calibration">
      <h3>Calibration</h3>
      <div class="cal-values">
        Min: <span id="calMin">---</span> | Max: <span id="calMax">---</span>
      </div>
      <div class="cal-buttons">
        <button class="btn-cal" onclick="calMin()">Set 0&deg;</button>
        <button class="btn-cal" onclick="calMax()">Set 360&deg;</button>
        <button class="btn-cal" onclick="calReset()">Reset</button>
      </div>
    </div>

    <div class="info">
      Firmware: <span id="firmware">---</span> | Uptime: <span id="uptime">---</span>
    </div>
  </div>

  <script>
    let isRotating = false;
    let currentState = 'IDLE';

    function updateStatus() {
      fetch('/status')
        .then(r => r.json())
        .then(data => {
          document.getElementById('heading').textContent = data.position_degrees.toFixed(1);
          document.getElementById('adc').textContent = data.position_adc;
          document.getElementById('needle').style.transform =
            'translateX(-50%) rotate(' + data.position_degrees + 'deg)';

          currentState = data.state;
          const stateEl = document.getElementById('state');
          stateEl.textContent = data.state;
          stateEl.className = 'status-value';
          if (data.state === 'IDLE') stateEl.classList.add('idle');
          else if (data.state.includes('ROTATING')) stateEl.classList.add('rotating');
          else if (data.state === 'EMERGENCY_STOP') stateEl.classList.add('emergency');

          document.getElementById('lastAction').textContent = data.last_action;
          document.getElementById('rssi').textContent = data.wifi_rssi + ' dBm';
          document.getElementById('calMin').textContent = data.adc_min;
          document.getElementById('calMax').textContent = data.adc_max;
          document.getElementById('firmware').textContent = data.firmware;
          document.getElementById('uptime').textContent = formatUptime(data.uptime);

          // Auto-rotate indicator
          const autoEl = document.getElementById('autoIndicator');
          if (data.auto_rotate) {
            autoEl.classList.add('active');
            document.getElementById('targetDisplay').textContent = data.target_heading.toFixed(1);
          } else {
            autoEl.classList.remove('active');
          }

          // Show/hide clear button
          document.getElementById('clearBtn').style.display =
            data.state === 'EMERGENCY_STOP' ? 'block' : 'none';
        })
        .catch(e => console.error('Status error:', e));
    }

    function formatUptime(seconds) {
      const h = Math.floor(seconds / 3600);
      const m = Math.floor((seconds % 3600) / 60);
      const s = seconds % 60;
      return h + 'h ' + m + 'm ' + s + 's';
    }

    function startCW() {
      if (currentState === 'EMERGENCY_STOP') return;
      isRotating = true;
      fetch('/cw').catch(e => console.error(e));
    }

    function startCCW() {
      if (currentState === 'EMERGENCY_STOP') return;
      isRotating = true;
      fetch('/ccw').catch(e => console.error(e));
    }

    function stop() {
      if (isRotating) {
        isRotating = false;
        fetch('/stop').catch(e => console.error(e));
      }
    }

    function emergency() {
      fetch('/emergency').catch(e => console.error(e));
    }

    function clearEmergency() {
      fetch('/clear').catch(e => console.error(e));
    }

    function goToHeading() {
      const heading = document.getElementById('targetHeading').value;
      if (heading === '' || heading < 0 || heading >= 360) {
        alert('Please enter a heading between 0 and 359');
        return;
      }
      fetch('/goto?heading=' + heading).catch(e => console.error(e));
    }

    function calMin() {
      if (confirm('Set current position as 0 degrees?')) {
        fetch('/cal/min').catch(e => console.error(e));
      }
    }

    function calMax() {
      if (confirm('Set current position as 360 degrees?')) {
        fetch('/cal/max').catch(e => console.error(e));
      }
    }

    function calReset() {
      if (confirm('Reset calibration to defaults?')) {
        fetch('/cal/reset').catch(e => console.error(e));
      }
    }

    // Start polling
    setInterval(updateStatus, 250);
    updateStatus();
  </script>
</body>
</html>
)rawliteral";

  return html;
}

// ============================================================================
// WIFI RESET BUTTON
// ============================================================================

void checkWiFiResetButton() {
  bool pressed = (digitalRead(WIFI_RESET_BUTTON) == LOW);

  if (pressed && !wifiResetButtonActive) {
    wifiResetButtonActive = true;
    wifiResetButtonPressTime = millis();
    Serial.println("[WIFI] Reset button held - release within 3s to cancel");
  }

  if (!pressed && wifiResetButtonActive) {
    wifiResetButtonActive = false;  // Released before threshold
  }

  if (wifiResetButtonActive && (millis() - wifiResetButtonPressTime >= WIFI_RESET_HOLD_MS)) {
    Serial.println("[WIFI] Resetting WiFi credentials and restarting...");
    WiFiManager wm;
    wm.resetSettings();
    delay(500);
    ESP.restart();
  }
}

// ============================================================================
// WIFI SETUP (WiFiManager)
// ============================================================================

void setupWiFi() {
  Serial.println("[WIFI] Starting WiFiManager...");

  WiFiManager wm;

  // Set timeout for configuration portal
  wm.setConfigPortalTimeout(180);  // 3 minutes

  // Custom parameters could be added here (hostname, etc.)

  // Try to connect, if fail, start config portal
  bool connected = wm.autoConnect(AP_NAME);

  if (!connected) {
    Serial.println("[WIFI] Failed to connect, restarting...");
    delay(3000);
    ESP.restart();
  }

  Serial.println("[WIFI] Connected!");
  Serial.print("[WIFI] IP Address: ");
  Serial.println(WiFi.localIP());

  // Disable WiFi sleep for better responsiveness
  WiFi.setSleep(false);
}

// ============================================================================
// MDNS SETUP
// ============================================================================

void setupMDNS() {
  if (MDNS.begin(MDNS_HOSTNAME)) {
    Serial.printf("[MDNS] Started: %s.local\n", MDNS_HOSTNAME);
    MDNS.addService("http", "tcp", WEB_SERVER_PORT);
  } else {
    Serial.println("[MDNS] Failed to start");
  }
}

// ============================================================================
// OTA SETUP
// ============================================================================

void setupOTA() {
  ArduinoOTA.setHostname(MDNS_HOSTNAME);

  ArduinoOTA.onStart([]() {
    // Stop all rotation before OTA update
    emergencyStop();
    Serial.println("[OTA] Update starting...");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA] Update complete!");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA] Progress: %u%%\r", (progress * 100) / total);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.printf("[OTA] Ready on port 3232\n");
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  CDE HAM-2 Rotor WiFi Controller");
  Serial.printf("  Firmware Version: %s\n", FIRMWARE_VERSION);
  Serial.println("========================================");
  Serial.println();

  // CRITICAL: Initialize relays OFF before anything else
  initializeRelays();

  // WiFi reset button
  pinMode(WIFI_RESET_BUTTON, INPUT_PULLUP);

  // Load calibration from EEPROM
  loadCalibration();

  // Initialize ADC
  analogReadResolution(ADC_RESOLUTION);
  pinMode(ADC_POSITION, INPUT);

  // Take initial position reading
  updatePosition();
  Serial.printf("[INIT] Initial position: %.1f degrees (ADC: %d)\n",
                currentPositionDegrees, currentPositionADC);

  // Setup networking
  setupWiFi();
  setupMDNS();
  setupOTA();
  setupWebServer();

  // Initialize state machine
  currentState = STATE_IDLE;
  stateStartTime = millis();

  Serial.println();
  Serial.println("[INIT] System ready!");
  Serial.printf("[INIT] Web interface: http://%s.local\n", MDNS_HOSTNAME);
  Serial.printf("[INIT] IP Address: %s\n", WiFi.localIP().toString().c_str());
  Serial.println();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long now = millis();

  // Check WiFi reset button
  checkWiFiResetButton();

  // Handle OTA updates
  ArduinoOTA.handle();

  // Handle web server requests
  server.handleClient();

  // Update position reading (at defined interval)
  if (now - lastPositionRead >= POSITION_READ_INTERVAL_MS) {
    updatePosition();
    lastPositionRead = now;
  }

  // Update state machine
  updateStateMachine();

  // Small delay to prevent watchdog issues
  delay(1);
}
