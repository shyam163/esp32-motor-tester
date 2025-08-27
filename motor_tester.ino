/*
 * =====================================================================================
 * ESP32 Motor Tester - Professional ESC Testing & Control System
 * =====================================================================================
 * 
 * Author: Shyam KJ
 * Created: 2024
 * Version: 1.0
 * License: MIT License
 * 
 * Description:
 * A comprehensive Electronic Speed Controller (ESC) testing tool designed for ESP32 
 * NodeMCU boards. Features a modern web-based interface supporting multiple communication
 * protocols including PWM and DShot (150/300/600). Provides real-time motor control,
 * bidirectional operation, safety features, and persistent configuration storage.
 * 
 * Key Features:
 * - Multi-protocol support (PWM, DShot150, DShot300, DShot600)
 * - Web-based responsive control interface
 * - Real-time throttle control with preset values
 * - Bidirectional motor control (DShot protocols)
 * - Safety arm/disarm system with emergency brake
 * - Configurable GPIO pin assignment (0-39)
 * - Persistent configuration storage in flash memory
 * - JSON-based REST API for programmatic control
 * - Comprehensive error handling and input validation
 * 
 * Hardware Requirements:
 * - ESP32 NodeMCU development board
 * - ESC compatible with selected protocol
 * - Brushless motor (for testing)
 * - Appropriate power supply
 * - Jumper wires for connections
 * 
 * Wiring:
 * ESP32 NodeMCU → ESC
 * GND          → GND
 * GPIO5 (default) → Signal (configurable via web interface)
 * 
 * Installation:
 * 1. Install required libraries: WiFi, WebServer, SPIFFS, ArduinoJson, Preferences
 * 2. Copy wifi_config.h.template to wifi_config.h and update with your WiFi credentials
 * 3. Upload SPIFFS data (index.html) using ESP32 Sketch Data Upload tool
 * 4. Upload sketch to ESP32
 * 5. Access web interface via ESP32's IP address
 * 
 * Safety Warnings:
 * ⚠️  ALWAYS remove propellers when testing motors
 * ⚠️  Secure motor before testing to prevent damage
 * ⚠️  Use appropriate power supply for ESC/motor combination
 * ⚠️  Start with minimum throttle and increase gradually
 * ⚠️  Always disarm when finished testing
 * 
 * Protocol Specifications:
 * 
 * PWM (Pulse Width Modulation):
 * - Signal range: 1000-2000 microseconds
 * - Update rate: 50Hz (20ms period)
 * - Resolution: ~1000 discrete steps
 * - Compatibility: Standard servo PWM protocol
 * 
 * DShot Protocol:
 * - Frame: 16 bits (11-bit throttle + 1-bit telemetry + 4-bit CRC)
 * - DShot150: 6.67μs bit period (150kbit/s)
 * - DShot300: 3.33μs bit period (300kbit/s)
 * - DShot600: 1.67μs bit period (600kbit/s)
 * - Resolution: 2048 discrete steps (0-2047)
 * - Features: Built-in error checking, bidirectional control, special commands
 * 
 * Web API Endpoints:
 * GET  /              - Main web interface
 * GET  /api/status    - System status (JSON)
 * POST /api/config    - Update configuration
 * POST /api/throttle  - Set throttle value
 * POST /api/direction - Set motor direction (DShot only)
 * POST /api/arm       - Arm ESC for operation
 * POST /api/disarm    - Disarm ESC (safety)
 * POST /api/brake     - Emergency brake (DShot only)
 * 
 * =====================================================================================
 */

#include <WiFi.h>        // ESP32 WiFi connectivity
#include <WebServer.h>   // HTTP web server functionality
#include <SPIFFS.h>      // Flash file system for web assets
#include <ArduinoJson.h> // JSON parsing and serialization
#include <Preferences.h> // Non-volatile storage for configuration
#include "wifi_config.h" // WiFi credentials (not tracked by git)

// =====================================================================================
// CONFIGURATION SECTION - Modify these values for your setup
// =====================================================================================

// WiFi Network Credentials (loaded from wifi_config.h)
// IMPORTANT: WiFi credentials are now stored in wifi_config.h file
// This file is excluded from git tracking for security purposes
const char* ssid = WIFI_SSID;             // WiFi network name from wifi_config.h
const char* password = WIFI_PASSWORD;     // WiFi password from wifi_config.h

// Web Server Configuration
WebServer server(80);                      // HTTP server on port 80
Preferences preferences;                   // Non-volatile storage interface

// =====================================================================================
// GLOBAL VARIABLES - System state and configuration
// =====================================================================================

// ESC Control Variables
// These variables maintain the current state of the motor control system
int escPin = 5;                        // GPIO pin for ESC signal output (configurable 0-39)
String protocol = "PWM";               // Active protocol: PWM, DSHOT150, DSHOT300, DSHOT600
int throttleValue = 1000;              // Current throttle: μs for PWM (1000-2000), raw for DShot (0-2047)
bool escArmed = false;                 // Safety state: true=armed (motor can run), false=disarmed
String direction = "FORWARD";          // Motor direction: FORWARD, REVERSE, BRAKE (DShot only)
bool directionSupported = false;       // Protocol capability flag for bidirectional control

// =====================================================================================
// DSHOT PROTOCOL CONSTANTS - Timing and command definitions
// =====================================================================================

// DShot Bit Timing Constants (in nanoseconds)
// These define the precise timing requirements for each DShot protocol variant
const int DSHOT150_BIT_TIME = 6667;   // 6.67μs bit period for DShot150 (150kbit/s)
const int DSHOT300_BIT_TIME = 3333;   // 3.33μs bit period for DShot300 (300kbit/s)  
const int DSHOT600_BIT_TIME = 1667;   // 1.67μs bit period for DShot600 (600kbit/s)

// DShot Bit Encoding Ratios
// DShot uses different pulse widths to encode binary 1s and 0s
const float DSHOT_HIGH_RATIO = 0.75;  // Bit 1: 75% of bit period HIGH, 25% LOW
const float DSHOT_LOW_RATIO = 0.375;  // Bit 0: 37.5% of bit period HIGH, 62.5% LOW

// DShot Special Commands (0-47 reserved for commands, 48-2047 for throttle values)
// These commands provide extended functionality beyond basic throttle control
const int DSHOT_CMD_MOTOR_STOP = 0;            // Emergency stop - immediately halt motor
const int DSHOT_CMD_BEEP1 = 1;                 // Audio feedback - short beep
const int DSHOT_CMD_BEEP2 = 2;                 // Audio feedback - double beep  
const int DSHOT_CMD_BEEP3 = 3;                 // Audio feedback - triple beep
const int DSHOT_CMD_BEEP4 = 4;                 // Audio feedback - quadruple beep
const int DSHOT_CMD_BEEP5 = 5;                 // Audio feedback - quintuple beep
const int DSHOT_CMD_ESC_INFO = 6;              // Request ESC information/telemetry
const int DSHOT_CMD_SPIN_DIRECTION_1 = 7;      // Set motor rotation direction (method 1)
const int DSHOT_CMD_SPIN_DIRECTION_2 = 8;      // Set motor rotation direction (method 2)
const int DSHOT_CMD_3D_MODE_OFF = 9;           // Disable 3D mode (bidirectional control)
const int DSHOT_CMD_3D_MODE_ON = 10;           // Enable 3D mode (bidirectional control)
const int DSHOT_CMD_SETTINGS_REQUEST = 11;     // Request current ESC settings
const int DSHOT_CMD_SAVE_SETTINGS = 12;        // Save current settings to ESC memory
const int DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20;     // Set normal (clockwise) rotation
const int DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21;   // Set reversed (counter-clockwise) rotation
const int DSHOT_CMD_LED0_ON = 22;              // Turn on LED channel 0 (if supported)
const int DSHOT_CMD_LED1_ON = 23;              // Turn on LED channel 1 (if supported)
const int DSHOT_CMD_LED2_ON = 24;              // Turn on LED channel 2 (if supported)
const int DSHOT_CMD_LED3_ON = 25;              // Turn on LED channel 3 (if supported)
const int DSHOT_CMD_LED0_OFF = 26;             // Turn off LED channel 0 (if supported)
const int DSHOT_CMD_LED1_OFF = 27;             // Turn off LED channel 1 (if supported)
const int DSHOT_CMD_LED2_OFF = 28;             // Turn off LED channel 2 (if supported)
const int DSHOT_CMD_LED3_OFF = 29;             // Turn off LED channel 3 (if supported)

// =====================================================================================
// MAIN SYSTEM FUNCTIONS - Setup and primary control loop
// =====================================================================================

/**
 * System Initialization Function
 * 
 * Performs complete system startup including:
 * - Serial communication setup
 * - File system initialization
 * - Configuration loading from flash memory
 * - GPIO pin configuration
 * - WiFi network connection
 * - Web server initialization
 * 
 * Called once at system boot/reset
 */
void setup() {
  // Initialize serial communication for debugging and status output
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Motor Tester Starting ===");
  Serial.println("Created by: Shyam KJ");
  Serial.println("Initializing system components...");
  
  // Initialize SPIFFS file system for web assets storage
  // The 'true' parameter enables formatting if mount fails
  if (!SPIFFS.begin(true)) {
    Serial.println("CRITICAL ERROR: SPIFFS mount failed!");
    Serial.println("Check if web files were uploaded using ESP32 Sketch Data Upload");
    return;
  }
  Serial.println("✓ SPIFFS file system initialized");
  
  // Initialize non-volatile storage for persistent configuration
  // Namespace "motor_tester" isolates our settings from other applications
  preferences.begin("motor_tester", false);
  Serial.println("✓ Preferences storage initialized");
  
  // Load previously saved configuration from flash memory
  // Uses default values if no saved configuration exists
  escPin = preferences.getInt("escPin", 5);                    // GPIO pin (default: 5)
  protocol = preferences.getString("protocol", "PWM");         // Protocol (default: PWM)
  direction = preferences.getString("direction", "FORWARD");   // Direction (default: FORWARD)
  
  Serial.printf("✓ Configuration loaded - Pin: %d, Protocol: %s, Direction: %s\n", 
                escPin, protocol.c_str(), direction.c_str());
  
  // Update protocol-specific capabilities
  updateDirectionSupport();
  
  // Configure ESC control GPIO pin
  // Set as output and ensure it starts in LOW (safe) state
  pinMode(escPin, OUTPUT);
  digitalWrite(escPin, LOW);
  Serial.printf("✓ ESC pin GPIO%d configured as output (LOW state)\n", escPin);
  
  // Establish WiFi connection
  Serial.printf("Connecting to WiFi network: %s\n", ssid);
  WiFi.begin(ssid, password);
  
  // Wait for WiFi connection with timeout protection
  int connectionAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && connectionAttempts < 30) {
    delay(1000);
    connectionAttempts++;
    Serial.printf("Connecting... attempt %d/30\n", connectionAttempts);
  }
  
  // Check connection status
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✓ WiFi connected successfully!");
    Serial.printf("✓ IP address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("✓ Access web interface at: http://%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("✗ WiFi connection failed!");
    Serial.println("Check WiFi credentials and network availability");
    return;
  }
  
  // Initialize web server with all API endpoints
  setupWebServer();
  Serial.println("✓ Web server routes configured");
  
  // Start HTTP server
  server.begin();
  Serial.println("✓ HTTP server started on port 80");
  Serial.println("=== System initialization complete ===\n");
}

/**
 * Main Program Loop
 * 
 * Executes continuously after setup() completes. Handles:
 * - Web server client requests (HTTP API calls)
 * - ESC signal generation based on current protocol
 * - Maintains 50Hz update rate for consistent motor control
 * 
 * Loop timing: 20ms cycle (50Hz) - standard for ESC control
 */
void loop() {
  // Process incoming HTTP requests from web interface
  // This handles all API endpoints (/api/status, /api/throttle, etc.)
  server.handleClient();
  
  // Generate ESC control signals only when system is armed and throttle > 0
  // Safety feature prevents accidental motor activation
  if (escArmed && throttleValue > 0) {
    if (protocol == "PWM") {
      // Send PWM signal (1000-2000μs pulse width)
      sendPWM(throttleValue);
    } else if (protocol.startsWith("DSHOT")) {
      // Send DShot digital signal (16-bit packet with CRC)
      sendDShot(throttleValue);
    }
  }
  
  // Maintain 50Hz update rate (20ms period)
  // Standard frequency for ESC control systems
  delay(20);
}

// =====================================================================================
// WEB SERVER CONFIGURATION - HTTP endpoints and request handling
// =====================================================================================

/**
 * Web Server Route Configuration
 * 
 * Sets up all HTTP endpoints for the web interface:
 * - Static file serving (HTML, CSS, JS)
 * - REST API endpoints for motor control
 * - JSON-based request/response handling
 */
void setupWebServer() {
  // Serve main web interface (index.html)
  // Root endpoint "/" serves the primary control interface
  server.on("/", HTTP_GET, []() {
    File file = SPIFFS.open("/index.html", "r");
    if (file) {
      server.streamFile(file, "text/html");
      file.close();
    } else {
      server.send(404, "text/plain", "Web interface file not found - check SPIFFS upload");
    }
  });
  
  // REST API Endpoints - JSON-based communication with web interface
  server.on("/api/status", HTTP_GET, handleGetStatus);        // Get system status
  server.on("/api/config", HTTP_POST, handleSetConfig);       // Update configuration
  server.on("/api/throttle", HTTP_POST, handleSetThrottle);   // Set motor throttle
  server.on("/api/direction", HTTP_POST, handleSetDirection); // Set motor direction (DShot)
  server.on("/api/arm", HTTP_POST, handleArm);                // Arm ESC (enable motor)
  server.on("/api/disarm", HTTP_POST, handleDisarm);          // Disarm ESC (safety)
  server.on("/api/brake", HTTP_POST, handleBrake);            // Emergency brake (DShot)
  
  // Static file server for additional web assets (CSS, JS, images)
  // Serves files directly from SPIFFS filesystem
  server.serveStatic("/", SPIFFS, "/");
}

// =====================================================================================
// API ENDPOINT HANDLERS - HTTP request processing functions
// =====================================================================================

/**
 * GET /api/status - System Status Handler
 * 
 * Returns current system state as JSON including:
 * - GPIO pin configuration
 * - Active protocol and capabilities
 * - Current throttle value and arm status
 * - Motor direction and support flags
 * - Network information
 * 
 * Used by web interface for real-time status updates
 */
void handleGetStatus() {
  DynamicJsonDocument doc(1024);
  
  // Hardware configuration
  doc["pin"] = escPin;                          // Active GPIO pin
  doc["protocol"] = protocol;                   // Current protocol
  doc["directionSupported"] = directionSupported; // Bidirectional capability
  
  // Motor control state
  doc["throttle"] = throttleValue;              // Current throttle setting
  doc["armed"] = escArmed;                      // Safety arm status
  doc["direction"] = direction;                 // Motor direction (DShot only)
  
  // Network information
  doc["ip"] = WiFi.localIP().toString();        // ESP32 IP address
  
  // Serialize JSON and send HTTP response
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

/**
 * POST /api/config - Configuration Update Handler
 * 
 * Updates system configuration with new settings:
 * - GPIO pin assignment (0-39)
 * - Protocol selection (PWM, DSHOT150, DSHOT300, DSHOT600)
 * 
 * Input JSON format: {"pin": 5, "protocol": "PWM"}
 * Changes are validated and saved to flash memory for persistence
 */
void handleSetConfig() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, server.arg("plain"));
    
    // Update GPIO pin configuration
    if (doc.containsKey("pin")) {
      int newPin = doc["pin"];
      // Validate GPIO pin range (ESP32 has GPIO 0-39)
      if (newPin >= 0 && newPin <= 39) {
        pinMode(escPin, INPUT);               // Release current pin
        escPin = newPin;                      // Update to new pin
        pinMode(escPin, OUTPUT);              // Configure new pin as output
        digitalWrite(escPin, LOW);            // Set safe initial state
        preferences.putInt("escPin", escPin); // Save to flash memory
        Serial.printf("Pin changed to GPIO%d\n", escPin);
      } else {
        Serial.printf("Invalid pin %d rejected (valid range: 0-39)\n", newPin);
      }
    }
    
    // Update protocol configuration
    if (doc.containsKey("protocol")) {
      String newProtocol = doc["protocol"];
      // Validate protocol selection
      if (newProtocol == "PWM" || newProtocol == "DSHOT150" || 
          newProtocol == "DSHOT300" || newProtocol == "DSHOT600") {
        protocol = newProtocol;                         // Update active protocol
        preferences.putString("protocol", protocol);    // Save to flash memory
        updateDirectionSupport();                       // Update protocol capabilities
        Serial.printf("Protocol changed to %s\n", protocol.c_str());
      } else {
        Serial.printf("Invalid protocol %s rejected\n", newProtocol.c_str());
      }
    }
    
    server.send(200, "application/json", "{\"status\":\"ok\"}");
  } else {
    server.send(400, "application/json", "{\"error\":\"No data received\"}");
  }
}

/**
 * POST /api/throttle - Throttle Control Handler
 * 
 * Sets motor throttle value with protocol-specific range validation:
 * - PWM: 1000-2000 microseconds (servo standard)
 * - DShot: 0-2047 raw values (48-2047 for throttle, 0-47 for commands)
 * 
 * Input JSON format: {"value": 1500}
 * Value is automatically constrained to valid range for active protocol
 */
void handleSetThrottle() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, server.arg("plain"));
    
    if (doc.containsKey("value")) {
      int value = doc["value"];
      
      if (protocol == "PWM") {
        // PWM protocol: 1000-2000 microseconds pulse width
        // 1000μs = minimum throttle, 2000μs = maximum throttle
        throttleValue = constrain(value, 1000, 2000);
        Serial.printf("PWM throttle set to %d μs\n", throttleValue);
      } else {
        // DShot protocol: 0-2047 raw values
        // 0-47: Special commands (motor stop, direction, etc.)
        // 48-2047: Throttle values (48=minimum, 2047=maximum)
        throttleValue = constrain(value, 0, 2047);
        Serial.printf("DShot throttle set to %d\n", throttleValue);
      }
    }
    
    server.send(200, "application/json", "{\"status\":\"ok\"}");
  } else {
    server.send(400, "application/json", "{\"error\":\"No data received\"}");
  }
}

/**
 * POST /api/arm - ESC Arm Handler
 * 
 * Arms the ESC for motor operation with safety protocol:
 * - Sets system to armed state (enables motor control)
 * - Initializes throttle to safe idle values
 * - PWM: 1000μs (minimum throttle)
 * - DShot: 48 (minimum throttle above command range)
 * 
 * SAFETY: Motor can only operate when armed
 */
void handleArm() {
  escArmed = true;
  
  // Set protocol-appropriate idle throttle values
  if (protocol == "PWM") {
    throttleValue = 1000;  // PWM idle: 1000μs (minimum safe value)
  } else {
    throttleValue = 48;    // DShot idle: 48 (above command range 0-47)
  }
  
  Serial.printf("ESC ARMED - Protocol: %s, Idle throttle: %d\n", 
                protocol.c_str(), throttleValue);
  server.send(200, "application/json", "{\"status\":\"armed\"}");
}

/**
 * POST /api/disarm - ESC Disarm Handler
 * 
 * Disarms the ESC for safety:
 * - Sets system to disarmed state (prevents motor operation)
 * - Sets throttle to zero
 * - Forces GPIO pin LOW (no signal output)
 * 
 * SAFETY: Always disarm when not actively testing
 */
void handleDisarm() {
  escArmed = false;              // Disable motor control
  throttleValue = 0;             // Zero throttle
  digitalWrite(escPin, LOW);     // Force pin LOW (no output signal)
  
  Serial.println("ESC DISARMED - Motor control disabled");
  server.send(200, "application/json", "{\"status\":\"disarmed\"}");
}

// =====================================================================================
// SIGNAL GENERATION FUNCTIONS - PWM and DShot protocol implementations
// =====================================================================================

/**
 * PWM Signal Generation Function
 * 
 * Generates standard servo PWM signal for ESC control:
 * - Signal period: 20ms (50Hz)
 * - Pulse width: 1000-2000μs (throttle range)
 * - Logic: HIGH for specified duration, LOW for remainder of period
 * 
 * @param microseconds Pulse width in microseconds (1000-2000)
 * 
 * PWM Timing:
 * - 1000μs = minimum throttle (motor stopped/idle)
 * - 1500μs = middle throttle (half speed)  
 * - 2000μs = maximum throttle (full speed)
 */
void sendPWM(int microseconds) {
  digitalWrite(escPin, HIGH);                    // Start pulse (rising edge)
  delayMicroseconds(microseconds);               // Hold HIGH for throttle duration
  digitalWrite(escPin, LOW);                     // End pulse (falling edge)
  delayMicroseconds(20000 - microseconds);       // Complete 20ms period (50Hz)
}

/**
 * DShot Signal Generation Function
 * 
 * Generates DShot digital protocol signals with precise timing:
 * - Frame: 16 bits total (11-bit throttle + 1-bit telemetry + 4-bit CRC)
 * - Bit encoding: Different pulse widths for 1s and 0s
 * - CRC validation: 4-bit checksum for error detection
 * - Interrupt disable: Ensures precise microsecond timing
 * 
 * @param value Throttle/command value (0-2047)
 *              0-47: Special commands (beep, direction, etc.)
 *              48-2047: Throttle values
 * 
 * DShot Bit Encoding:
 * - Bit 1: 75% HIGH, 25% LOW of bit period
 * - Bit 0: 37.5% HIGH, 62.5% LOW of bit period
 * 
 * Protocol Speeds:
 * - DShot150: 6.67μs bit period
 * - DShot300: 3.33μs bit period  
 * - DShot600: 1.67μs bit period
 */
void sendDShot(int value) {
  // Step 1: Create 16-bit DShot packet structure
  // Bits 15-5: 11-bit throttle/command value
  // Bit 4: Telemetry request bit (0 = no telemetry requested)
  uint16_t packet = (value << 1);  
  
  // Step 2: Calculate 4-bit CRC checksum
  // CRC protects against transmission errors
  uint16_t crc = 0;
  uint16_t temp = packet;
  for (int i = 0; i < 3; i++) {
    crc ^= (temp & 0xF);    // XOR each 4-bit nibble
    temp >>= 4;             // Shift to next nibble
  }
  packet |= (crc << 12);    // Add CRC to bits 15-12
  
  // Step 3: Determine timing parameters for current DShot protocol
  int bitTime = DSHOT150_BIT_TIME;                    // Default to DShot150
  if (protocol == "DSHOT300") bitTime = DSHOT300_BIT_TIME;
  else if (protocol == "DSHOT600") bitTime = DSHOT600_BIT_TIME;
  
  // Convert nanoseconds to microseconds for delayMicroseconds()
  int highTime1 = bitTime * DSHOT_HIGH_RATIO / 1000; // Bit 1 HIGH duration
  int lowTime1 = bitTime - highTime1;                 // Bit 1 LOW duration
  int highTime0 = bitTime * DSHOT_LOW_RATIO / 1000;  // Bit 0 HIGH duration  
  int lowTime0 = bitTime - highTime0;                 // Bit 0 LOW duration
  
  // Step 4: Disable interrupts for precise timing
  // Critical for maintaining DShot timing accuracy
  noInterrupts();
  
  // Step 5: Transmit 16-bit packet (MSB first)
  for (int i = 15; i >= 0; i--) {
    if (packet & (1 << i)) {
      // Transmit bit 1: Long HIGH pulse, short LOW pulse
      digitalWrite(escPin, HIGH);
      delayMicroseconds(highTime1);
      digitalWrite(escPin, LOW);
      delayMicroseconds(lowTime1);
    } else {
      // Transmit bit 0: Short HIGH pulse, long LOW pulse
      digitalWrite(escPin, HIGH);
      delayMicroseconds(highTime0);
      digitalWrite(escPin, LOW);
      delayMicroseconds(lowTime0);
    }
  }
  
  // Step 6: Re-enable interrupts
  interrupts();
}

// =====================================================================================
// UTILITY FUNCTIONS - Protocol management and helper functions
// =====================================================================================

/**
 * Protocol Capability Update Function
 * 
 * Updates bidirectional control support based on active protocol:
 * - DShot protocols: Support bidirectional control (FORWARD/REVERSE/BRAKE)
 * - PWM protocol: Unidirectional only (FORWARD)
 * 
 * Called when protocol is changed via configuration
 */
void updateDirectionSupport() {
  // Check if current protocol supports bidirectional control
  directionSupported = protocol.startsWith("DSHOT");
  
  if (!directionSupported) {
    direction = "FORWARD";  // PWM only supports forward direction
    Serial.println("Direction control disabled (PWM mode)");
  } else {
    Serial.println("Direction control enabled (DShot mode)");
  }
}

/**
 * POST /api/direction - Motor Direction Handler (DShot only)
 * 
 * Sets motor rotation direction using DShot special commands:
 * - FORWARD: Normal rotation (clockwise)
 * - REVERSE: Reversed rotation (counter-clockwise)  
 * - BRAKE: Active braking (not implemented in this handler)
 * 
 * Input JSON format: {"direction": "FORWARD"}
 * Only available when using DShot protocols
 */
void handleSetDirection() {
  // Validate protocol support
  if (!directionSupported) {
    server.send(400, "application/json", "{\"error\":\"Direction control not supported by current protocol\"}");
    return;
  }
  
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, server.arg("plain"));
    
    if (doc.containsKey("direction")) {
      String newDirection = doc["direction"];
      
      // Validate direction parameter
      if (newDirection == "FORWARD" || newDirection == "REVERSE" || newDirection == "BRAKE") {
        direction = newDirection;
        preferences.putString("direction", direction); // Save to flash memory
        
        // Send appropriate DShot command to ESC
        if (direction == "FORWARD") {
          sendDShotCommand(DSHOT_CMD_SPIN_DIRECTION_NORMAL);
          Serial.println("Motor direction set to FORWARD");
        } else if (direction == "REVERSE") {
          sendDShotCommand(DSHOT_CMD_SPIN_DIRECTION_REVERSED);
          Serial.println("Motor direction set to REVERSE");
        }
        
        server.send(200, "application/json", "{\"status\":\"ok\"}");
      } else {
        server.send(400, "application/json", "{\"error\":\"Invalid direction\"}");
      }
    } else {
      server.send(400, "application/json", "{\"error\":\"No direction specified\"}");
    }
  } else {
    server.send(400, "application/json", "{\"error\":\"No data received\"}");
  }
}

/**
 * POST /api/brake - Emergency Brake Handler (DShot only)
 * 
 * Applies emergency brake to stop motor immediately:
 * - Sends DShot MOTOR_STOP command
 * - Sets throttle to zero
 * - Updates direction status to BRAKE
 * 
 * Requires system to be armed for safety validation
 * Only available with DShot protocols
 */
void handleBrake() {
  // Validate protocol support
  if (!directionSupported) {
    server.send(400, "application/json", "{\"error\":\"Braking not supported by current protocol\"}");
    return;
  }
  
  // Validate system is armed (safety check)
  if (escArmed) {
    direction = "BRAKE";                        // Update direction status
    throttleValue = 0;                          // Zero throttle
    sendDShotCommand(DSHOT_CMD_MOTOR_STOP);     // Send emergency stop command
    
    Serial.println("EMERGENCY BRAKE APPLIED - Motor stopped");
    server.send(200, "application/json", "{\"status\":\"braking\"}");
  } else {
    server.send(400, "application/json", "{\"error\":\"ESC must be armed to brake\"}");
  }
}

/**
 * DShot Command Transmission Function
 * 
 * Sends DShot special commands to ESC with reliability enhancement:
 * - Transmits command 10 times for reliable reception
 * - Uses 1ms delay between transmissions
 * - Only operates with DShot protocols
 * 
 * @param command DShot command value (0-47)
 * 
 * Common commands:
 * - 0: Motor stop
 * - 20: Normal direction
 * - 21: Reversed direction
 * - 1-5: Audio beeps
 */
void sendDShotCommand(int command) {
  if (!protocol.startsWith("DSHOT")) return;
  
  Serial.printf("Sending DShot command: %d\n", command);
  
  // Send command multiple times for reliable ESC reception
  // ESCs typically require 3-10 repetitions to register commands
  for (int i = 0; i < 10; i++) {
    sendDShot(command);
    delay(1);  // Brief delay between transmissions
  }
}

// =====================================================================================
// END OF ESP32 MOTOR TESTER CODE
// Created by: Shyam KJ
// =====================================================================================