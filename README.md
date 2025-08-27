# ESP32 Motor Tester

A comprehensive ESC (Electronic Speed Controller) testing tool for ESP32 NodeMCU boards with web interface support for multiple protocols including PWM and DShot.

## Features

- **Web-based Interface**: Modern responsive UI accessible from any device
- **Multiple Protocols**: Support for PWM, DShot150, DShot300, and DShot600
- **Pin Configuration**: Reassignable output pins via web interface
- **Real-time Control**: Live throttle control with preset values
- **Persistent Settings**: Configuration saved to ESP32 flash memory
- **Safety Features**: Arm/Disarm functionality for safe operation

## Supported Protocols

### PWM (Pulse Width Modulation)
- Standard servo PWM protocol
- Range: 1000-2000 microseconds
- 50Hz update rate
- Compatible with most ESCs

### DShot Protocol Support
- **DShot150**: 150kbit/s data rate
- **DShot300**: 300kbit/s data rate  
- **DShot600**: 600kbit/s data rate
- 11-bit throttle resolution (0-2047)
- Built-in CRC error checking
- No calibration required

## Hardware Requirements

- ESP32 NodeMCU development board
- ESC compatible with chosen protocol
- Brushless motor (for testing)
- Power supply for ESC/motor
- Jumper wires for connections

## Wiring

```
ESP32 NodeMCU → ESC
GND          → GND
GPIO5 (default) → Signal
```

**Note**: Default signal pin is GPIO5, but can be changed via web interface to any GPIO pin (0-39).

## Installation

1. Install required libraries in Arduino IDE:
   - WiFi library (built-in)
   - WebServer library (built-in) 
   - ArduinoJson library
   - SPIFFS library (built-in)

2. Upload filesystem (SPIFFS):
   - Install ESP32 Sketch Data Upload tool
   - Select Tools → ESP32 Sketch Data Upload
   - Upload the data folder contents to SPIFFS

3. Configure WiFi credentials:
   - Edit `motor_tester.ino`
   - Change `ssid` and `password` variables

4. Upload sketch to ESP32

## Usage

1. **Connect to WiFi**: ESP32 will connect to configured network
2. **Access Web Interface**: Navigate to ESP32's IP address in browser
3. **Configure Protocol**: Select desired protocol (PWM/DShot)
4. **Set Output Pin**: Choose GPIO pin for ESC signal
5. **Arm ESC**: Click "Arm ESC" to enable motor control
6. **Control Throttle**: Use slider or preset buttons
7. **Disarm**: Click "Disarm ESC" to stop motor safely

## Web Interface Features

### Status Panel
- Current arm/disarm status
- Active protocol
- Output pin number
- ESP32 IP address

### Configuration
- Protocol selection dropdown
- Pin number input (0-39)
- Save configuration button

### Motor Control
- Real-time throttle display
- Smooth slider control
- Quick preset buttons (Min, 25%, 50%, 75%, Max)
- Arm/Disarm safety controls

## Safety Warnings

⚠️ **IMPORTANT SAFETY NOTES**:

1. **Always remove propellers** when testing motors
2. **Secure motor** before testing to prevent damage
3. **Use appropriate power supply** for your ESC/motor combination
4. **Start with minimum throttle** and increase gradually
5. **Always disarm** when finished testing
6. **Double-check connections** before powering on

## Protocol Details

### PWM Timing
- Signal: 1000-2000μs pulse width
- Period: 20ms (50Hz)
- Resolution: ~1000 steps

### DShot Timing
- Frame: 16 bits (11-bit throttle + 1-bit telemetry + 4-bit CRC)
- DShot150: 6.67μs bit period
- DShot300: 3.33μs bit period  
- DShot600: 1.67μs bit period
- Resolution: 2048 steps (0-2047)

## Troubleshooting

### ESP32 won't connect to WiFi
- Check SSID/password in code
- Ensure 2.4GHz WiFi network
- Check serial monitor for connection status

### Web interface not loading
- Verify SPIFFS upload completed successfully
- Check ESP32 IP address in serial monitor
- Try accessing by IP directly

### Motor not responding
- Verify ESC compatibility with selected protocol
- Check wiring connections
- Ensure ESC is properly calibrated (for PWM)
- Try different GPIO pin

### DShot not working
- Ensure ESC supports DShot protocol
- Try lower DShot speed (150 → 300 → 600)
- Check timing precision (disable other interrupts)

## License

This project is open source and available under the MIT License.