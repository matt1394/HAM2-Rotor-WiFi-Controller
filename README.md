# CDE HAM-2 Antenna Rotor WiFi Controller

WiFi remote control system for a CDE HAM-2 antenna rotor with safety-first design principles for mechanical control systems.

## Features

- **Web Interface**: Real-time compass display, manual CW/CCW controls, automatic go-to-heading
- **Safety State Machine**: Enforced brake release timing (1s before rotation, 1.5s after)
- **Position Feedback**: ADC-based position reading with averaging and calibration
- **WiFiManager**: No hardcoded credentials - configure via captive portal on first boot
- **OTA Updates**: Update firmware over WiFi
- **mDNS**: Access at `http://rotor-control.local`

## Hardware

- **Microcontroller**: ESP32 Dev Module with integrated 4-channel relay module
- **Relay Logic**: Active LOW (LOW = ON, HIGH = OFF)
- **GPIO Mapping**:
  - GPIO 32: Brake Release (Relay 1)
  - GPIO 33: Clockwise Rotation (Relay 2)
  - GPIO 25: Counter-Clockwise Rotation (Relay 3)
  - GPIO 26: Spare (Relay 4)
  - GPIO 34: ADC Position Input
- **Position Feedback**: 470kΩ/100kΩ voltage divider from rotor meter (0-12.6V → 0-2.2V)

## Safety Features

- 1 second brake release delay before rotation starts
- 1.5 second delay after rotation stops before brake re-engages
- 60 second maximum continuous rotation timeout
- Emergency stop with immediate halt
- All relays initialize to OFF on startup

## Installation

### Required Libraries

Install via Arduino Library Manager:
- WiFiManager (by tzapu)

Built-in libraries (no installation needed):
- WiFi, WebServer, ESPmDNS, ArduinoOTA, EEPROM

### Arduino IDE Settings

- **Board**: ESP32 Dev Module
- **Upload Speed**: 115200
- **Partition Scheme**: Default (with OTA)

### First-Time Setup

1. Upload firmware via USB
2. Connect to "Rotor-Controller-Setup" WiFi AP
3. Configure your home WiFi in the captive portal
4. Access web interface at `http://rotor-control.local`

## Calibration

The CDE HAM-2 indicator starts at South (180°) at 0V and sweeps CW through 360° back to South at ~12.5V.

1. Manually rotate rotor to 0° (North) - this is mid-scale on the meter
2. Note: Calibration is set via web interface "Set 0°" and "Set 360°" buttons
3. Position at minimum voltage (South/180°) and click "Set 0°" (represents start of sweep)
4. Position at maximum voltage (South/180° after full rotation) and click "Set 360°"

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Web interface |
| `/status` | GET | JSON status data |
| `/cw` | GET | Start clockwise rotation |
| `/ccw` | GET | Start counter-clockwise rotation |
| `/stop` | GET | Stop rotation |
| `/emergency` | GET | Emergency stop |
| `/clear` | GET | Clear emergency stop |
| `/goto?heading=N` | GET | Auto-rotate to heading (0-359) |
| `/cal/min` | GET | Set current ADC as 0° |
| `/cal/max` | GET | Set current ADC as 360° |
| `/cal/reset` | GET | Reset calibration to defaults |

## License

MIT License
