#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include "Adafruit_MCP9601.h"
#include <EEPROM.h>

// Pin Definitions
const int PRESSURE_PIN = 22;     // Analog input for pressure transducer
const int SWITCH_PIN = 16;       // Switch for starting/stopping recording
const int LED_PIN = 15;          // LED indicator
const int SD_CHIP_SELECT = BUILTIN_SDCARD;  // Teensy 4.1 built-in SD card

// I2C Addresses
const uint8_t MCP9601_I2C_ADDR = 0x67;  // Default I2C address for the MCP9601

// System parameters
const int LED_BRIGHTNESS_STANDBY = 1;   // Very dim in standby
const int LED_BRIGHTNESS_ACTIVE = 50;   // Brighter when recording
const int LED_BRIGHTNESS_ERROR = 150;   // Very bright for errors/warnings

// Pressure transducer parameters
const float PRESSURE_MAX = 2500.0;      // Maximum pressure in PSI
const float VOLTAGE_MIN = 0.3;         // Voltage at 0 PSI (0.33V)
const float VOLTAGE_MAX = 3.0;          // Voltage at max PSI (3.0V)
const float VOLTAGE_RANGE = VOLTAGE_MAX - VOLTAGE_MIN;  // Voltage range

// EEPROM addresses for storing calibration data
const int EEPROM_ADDR_CALIBRATED = 0;     // Flag to indicate if calibration data exists (1 byte)
const int EEPROM_ADDR_ZERO_OFFSET = 4;    // Zero offset value (4 bytes - float)
const int EEPROM_ADDR_CALIB_FACTOR = 8;   // Calibration factor (4 bytes - float)

// Load cell calibration variables
float zeroOffset = 0;                     // Raw value when scale is empty
float calibrationFactor = 1.0;            // Factor to convert raw reading to weight
bool isCalibrated = false;                // Flag to track if calibration is complete

// Global objects
NAU7802 scaleHandler;            // Object for the Qwiic Scale
Adafruit_MCP9601 thermocouple;   // MCP9601 thermocouple interface
Ambient_Resolution ambientRes = RES_ZERO_POINT_0625;
File dataFile;                   // File object for logging data
bool isRecording = false;        // Recording state
String filename;                 // Dynamic filename for the log
bool thermocoupleWorking = false;  // Track if thermocouple is working

// SD card write buffer to optimize performance
const int BUFFER_SIZE = 4096;    // 4KB buffer for SD card writes
char dataBuffer[BUFFER_SIZE];    // Buffer to hold data before writing to SD
int bufferPos = 0;               // Current position in buffer

// Switch state tracking
bool initialSwitchState;         // Store the initial state of the switch
bool previousSwitchState;        // Track previous state for change detection

unsigned long lastFlushTime = 0;
const unsigned long FLUSH_INTERVAL_MS = 500; // Flush buffer every 500ms

// Function prototypes
void testThermocouple();
void readWeight();
void displayRawReadings(unsigned long duration);
void tareScale();
void calibrateScale();
void saveCalibrationData();
void loadCalibrationData();
void startRecording();
void stopRecording();
void collectAndLogData();
void addToBuffer(const char* data);
void flushBuffer();
void createFilename();
void errorBlink(int errorCode);
void testPressureTransducer();
float calculatePressure(float voltage);

// Function to calculate pressure from voltage with offset compensation
float calculatePressure(float voltage) {
  // Check if voltage is below minimum - if so, return 0 pressure
  if (voltage < VOLTAGE_MIN) {
    return 0.0;
  }
  
  // Calculate pressure based on linear relationship
  // where VOLTAGE_MIN = 0 PSI and VOLTAGE_MAX = PRESSURE_MAX PSI
  float normalizedVoltage = voltage - VOLTAGE_MIN;  // Subtract offset
  float pressurePercent = normalizedVoltage / VOLTAGE_RANGE;  // Get percentage of range
  float pressure = pressurePercent * PRESSURE_MAX;  // Convert to PSI
  
  return pressure;
}
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(500); // Brief delay for stability
  Serial.println("\nData Acquisition System Starting...");
  
  // Initialize pins
  pinMode(SWITCH_PIN, INPUT_PULLUP);  // Switch with pullup
  pinMode(LED_PIN, OUTPUT);           // LED output
  analogWriteResolution(8);           // 8-bit PWM resolution
  analogWrite(LED_PIN, 0);            // LED off initially
  
  // Read the initial switch state
  delay(10); // Small delay to stabilize input
  initialSwitchState = digitalRead(SWITCH_PIN);
  previousSwitchState = initialSwitchState;
  Serial.print("Initial switch state: ");
  Serial.println(initialSwitchState ? "HIGH" : "LOW");
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(100000);  // Start with 100kHz for device initialization
  
  // Initialize scale
  if (!scaleHandler.begin()) {
    Serial.println("Qwiic Scale not detected!");
    errorBlink(2);
  } else {
    scaleHandler.setGain(NAU7802_GAIN_64);
    scaleHandler.setSampleRate(NAU7802_SPS_320); // Fastest sample rate
    scaleHandler.calibrateAFE();
    Serial.println("Scale initialized");
    
    // Load calibration data from EEPROM if available
    loadCalibrationData();
    
    // If not calibrated, offer calibration
    if (!isCalibrated) {
      Serial.println("Scale not calibrated. Enter 'c' to calibrate now or any other key to continue.");
      unsigned long startTime = millis();
      while (millis() - startTime < 5000) { // Wait for 5 seconds for user input
        if (Serial.available()) {
          char userInput = Serial.read();
          if (userInput == 'c' || userInput == 'C') {
            calibrateScale();
            break;
          } else {
            Serial.println("Continuing without calibration.");
            break;
          }
        }
        // Blink LED while waiting
        analogWrite(LED_PIN, (millis() % 500 < 250) ? LED_BRIGHTNESS_ERROR : 0);
      }
    } else {
      Serial.println("Scale calibration loaded:");
      Serial.print("Zero Offset: ");
      Serial.println(zeroOffset);
      Serial.print("Calibration Factor: ");
      Serial.println(calibrationFactor);
    }
  }
  
  // Try to initialize MCP9601, but don't halt if it fails
  Serial.println("Attempting to initialize MCP9601...");
  thermocouple.begin(MCP9601_I2C_ADDR);
  if (thermocouple.begin(MCP9601_I2C_ADDR)) {
    // Configure using MCP9600 commands
    thermocouple.setADCresolution(MCP9600_ADCRESOLUTION_14);
    thermocouple.setAmbientResolution(RES_ZERO_POINT_25);
    thermocouple.setFilterCoefficient(0);
    thermocouple.setThermocoupleType(MCP9600_TYPE_K);
    thermocouple.begin(MCP9601_I2C_ADDR);
    // Set up continuous conversion mode for the thermocouple
    //thermocouple.setConversionMode(MCP9601_CONTINUOUS_CONVERSION);
    //thermocouple.setConversionFrequency(MCP9601_CONV_FREQ_30HZ);
    thermocouple.enable(true);
    
    thermocoupleWorking = true;
    Serial.println("MCP9601 initialized successfully");
  } else {
    thermocoupleWorking = false;
    Serial.println("MCP9601 not detected or not responding - continuing without temperature data");
    
    // Warning blink (3 quick blinks)
    for (int i = 0; i < 3; i++) {
      analogWrite(LED_PIN, LED_BRIGHTNESS_ERROR);
      delay(100);
      analogWrite(LED_PIN, 0);
      delay(100);
    }
  }
  
  // Increase I2C speed for operation after initialization - max 400kHz for Qwiic devices
  Wire.setClock(400000);
  
  // Set ADC resolution for pressure readings
  analogReadResolution(12);  // Teensy 4.1 has 12-bit ADC
  
  // Test pressure transducer
  int rawPressure = analogRead(PRESSURE_PIN);
  float voltage = (rawPressure * 3.3) / 4095.0;
  float pressure = calculatePressure(voltage);
  Serial.println("Pressure transducer test:");
  Serial.print("  Raw ADC: ");
  Serial.print(rawPressure);
  Serial.print(", Voltage: ");
  Serial.print(voltage, 3);
  Serial.print("V, Pressure: ");
  Serial.print(pressure, 1);
  Serial.println(" PSI");
  
  // Initialize SD card with optimized settings
  if (!SD.begin(SD_CHIP_SELECT)) {
    Serial.println("SD card initialization failed!");
    errorBlink(4);
  } else {
    Serial.println("SD card initialized.");
    SPI.setClockDivider(SPI_CLOCK_DIV2); // Maximize SPI speed for SD card
  }
  
  // Generate a unique filename
  createFilename();
  
  // Indicate ready state with very dim LED
  analogWrite(LED_PIN, LED_BRIGHTNESS_STANDBY);
  
  Serial.println("System ready. Change switch state to start recording.");
  Serial.println("\n=== Available Commands ===");
  Serial.println("c - Calibrate scale");
  Serial.println("t - Tare scale (set zero point)");
  Serial.println("h - Read Thermocouple");
  Serial.println("r - Read current weight");
  Serial.println("d - Display raw readings for 5 seconds");
  Serial.println("p - Test pressure transducer");
  Serial.println("s - Start/stop recording manually");
  Serial.println("==========================");
}

void loop() {
  // Check for serial commands
  if (Serial.available()) {
    char userInput = Serial.read();
    
    switch (userInput) {
      case 'c': 
      case 'C':
        calibrateScale();
        break;
        
      case 't': 
      case 'T':
        tareScale();
        break;
     
      case 'h':  // 'h' for heat (thermocouple)
      case 'H':
        testThermocouple();
        break;
        
      case 'r': 
      case 'R':
        readWeight();
        break;
        
      case 'd':
      case 'D':
        displayRawReadings(5000); // Display for 5 seconds
        break;
      
      case 'p':
      case 'P':
        // Test pressure transducer
        testPressureTransducer();
        break;
        
      case 's':
      case 'S':
        if (isRecording) {
          stopRecording();
        } else {
          startRecording();
        }
        break;
    }
  }

  // Read current switch state
  bool currentSwitchState = digitalRead(SWITCH_PIN);
  
  // Detect switch state change - compare with initial state
  if (currentSwitchState != previousSwitchState) {
    delay(50); // Simple debounce
    currentSwitchState = digitalRead(SWITCH_PIN);
    
    // If the state is still different after debounce
    if (currentSwitchState != previousSwitchState) {
      // Only toggle recording if the state is different from initial state
      if (currentSwitchState != initialSwitchState) {
        if (!isRecording) {
          startRecording();
        } else {
          stopRecording();
        }
      } else {
        // Switch returned to initial state, stop recording if active
        if (isRecording) {
          stopRecording();
        }
      }
      
      previousSwitchState = currentSwitchState;
    }
  }
  
  // Always collect data at maximum speed when recording
  if (isRecording) {

    collectAndLogData();
    
    // Periodically flush buffer to SD card
    unsigned long currentTime = millis();
    if ((currentTime - lastFlushTime >= FLUSH_INTERVAL_MS) || (bufferPos > BUFFER_SIZE - 200)) {
      flushBuffer();
      lastFlushTime = currentTime;
    }
  }
}

// Read current weight with detailed debug info
void readWeight() {
  Serial.println("\n=== WEIGHT READING ===");
  
  // Check if scale is available
  if (!scaleHandler.available()) {
    Serial.println("ERROR: Scale not available!");
    return;
  }
  
  // Take several readings and average for stability
  long rawTotal = 0;
  int samples = 5;
  
  Serial.println("Raw readings:");
  for (int i = 0; i < samples; i++) {
    while (!scaleHandler.available()) delay(10);
    long reading = scaleHandler.getReading();
    rawTotal += reading;
    Serial.print("  ");
    Serial.println(reading);
    delay(50);
  }
  
  long rawReading = rawTotal / samples;
  
  // Display all the debug info
  Serial.println("\nCalibration Parameters:");
  Serial.print("  Zero offset: ");
  Serial.println(zeroOffset);
  Serial.print("  Calibration factor: ");
  Serial.println(calibrationFactor);
  
  Serial.println("\nCalculations:");
  Serial.print("  Average raw reading: ");
  Serial.println(rawReading);
  Serial.print("  Difference (raw - zero): ");
  float difference = (float)(rawReading - zeroOffset);
  Serial.println(difference);
  Serial.print("  Calculation: ");
  Serial.print(difference);
  Serial.print(" × ");
  Serial.print(calibrationFactor);
  Serial.print(" = ");
  
  // Calculate weight
  float weight = calibrationFactor * difference;
  Serial.print(weight);
  Serial.println(" g");
}

// Display raw readings for a specified time
void displayRawReadings(unsigned long duration) {
  Serial.println("\n=== Raw Readings ===");
  Serial.println("Showing raw readings for " + String(duration / 1000) + " seconds...");
  
  unsigned long endTime = millis() + duration;
  unsigned long lastPrintTime = 0;
  const unsigned long PRINT_INTERVAL = 100; // 10 readings per second
  
  while (millis() < endTime) {
    unsigned long currentTime = millis();
    
    if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
      if (scaleHandler.available()) {
        long rawReading = scaleHandler.getReading();
        Serial.println(rawReading);
        lastPrintTime = currentTime;
      }
    }
  }
  
  Serial.println("=== End of Raw Readings ===");
}

// Tare the scale (set zero point)
void tareScale() {
  Serial.println("\n=== TARE SCALE ===");
  Serial.println("Setting current reading as zero point...");
  
  // Take several readings and average for stability
  long zeroSum = 0;
  int samples = 10;
  
  for (int i = 0; i < samples; i++) {
    while (!scaleHandler.available()) delay(10);
    long reading = scaleHandler.getReading();
    zeroSum += reading;
    Serial.print("Sample ");
    Serial.print(i+1);
    Serial.print(": ");
    Serial.println(reading);
    delay(100);
  }
  
  // Set new zero offset
  zeroOffset = (float)zeroSum / samples;
  
  // Save to EEPROM
  saveCalibrationData();
  
  Serial.print("New zero offset: ");
  Serial.println(zeroOffset);
}

// Test pressure transducer with detailed information
void testPressureTransducer() {
  Serial.println("\n=== PRESSURE TRANSDUCER TEST ===");
  Serial.println("Taking multiple readings to verify pressure transducer...");
  
  for (int i = 0; i < 10; i++) {
    int rawPressure = analogRead(PRESSURE_PIN);
    float voltage = (rawPressure * 3.3) / 4095.0;
    float pressure = calculatePressure(voltage);
    
    Serial.print("Reading ");
    Serial.print(i+1);
    Serial.print(": Raw ADC = ");
    Serial.print(rawPressure);
    Serial.print(", Voltage = ");
    Serial.print(voltage, 3);
    Serial.print("V (");
    
    if (voltage < VOLTAGE_MIN) {
      Serial.print("below min ");
    } else if (voltage > VOLTAGE_MAX) {
      Serial.print("above max ");
    } else {
      float percent = ((voltage - VOLTAGE_MIN) / VOLTAGE_RANGE) * 100.0;
      Serial.print(percent, 1);
      Serial.print("% ");
    }
    
    Serial.print("of range), Pressure = ");
    Serial.print(pressure, 1);
    Serial.println(" PSI");
    
    delay(200); // Short delay between readings
  }
  
  Serial.println("Pressure transducer test complete.");
  Serial.println("Expected range: 0.33V (0 PSI) to 3.00V (2500 PSI)");
}

// Calibrate scale with known weight
void calibrateScale() {
  Serial.println("\n=== SCALE CALIBRATION ===");
  
  // Step 1: Set zero point
  Serial.println("Step 1: Remove all weight from scale and press Enter");
  while (Serial.available()) Serial.read(); // Clear any existing input
  while (!Serial.available()) delay(10);
  Serial.read(); // Clear the Enter key
  
  // Get zero readings
  Serial.println("Taking zero readings...");
  long zeroSum = 0;
  int samples = 10;
  
  for (int i = 0; i < samples; i++) {
    while (!scaleHandler.available()) delay(10);
    long reading = scaleHandler.getReading();
    zeroSum += reading;
    Serial.print("Zero sample ");
    Serial.print(i+1);
    Serial.print(": ");
    Serial.println(reading);
    delay(100);
  }
  
  zeroOffset = (float)zeroSum / samples;
  Serial.print("Zero offset set to: ");
  Serial.println(zeroOffset);
  
  // Step 2: Get known weight
  Serial.println("\nStep 2: Place a known weight on the scale and enter weight in grams:");
  
  while (Serial.available()) Serial.read(); // Clear existing input
  
  String weightInput = "";
  while (!Serial.available()) delay(10);
  
  while (Serial.available()) {
    char c = Serial.read();
    if (isdigit(c) || c == '.') {
      weightInput += c;
      Serial.print(c); // Echo character for feedback
    } else if (c == '\n' || c == '\r') {
      break;
    }
    delay(2);
  }
  
  Serial.println(); // New line after input
  float knownWeight = weightInput.toFloat();
  
  if (knownWeight <= 0) {
    Serial.println("Invalid weight value! Please enter a number greater than zero.");
    return;
  }
  
  Serial.print("Using known weight: ");
  Serial.print(knownWeight);
  Serial.println(" g");
  
  // Take readings with known weight
  Serial.println("Taking weight readings...");
  long weightSum = 0;
  
  for (int i = 0; i < samples; i++) {
    while (!scaleHandler.available()) delay(10);
    long reading = scaleHandler.getReading();
    weightSum += reading;
    Serial.print("Weight sample ");
    Serial.print(i+1);
    Serial.print(": ");
    Serial.println(reading);
    delay(100);
  }
  
  float weightReading = (float)weightSum / samples;
  
  // Calculate and verify calibration factor
  float difference = weightReading - zeroOffset;
  Serial.print("Reading difference (weight - zero): ");
  Serial.println(difference);
  
  if (difference <= 0) {
    Serial.println("ERROR: Weight reading must be greater than zero reading!");
    Serial.println("Make sure weight is properly applied to the scale.");
    return;
  }
  
  // Calculate calibration factor
  calibrationFactor = knownWeight / difference;
  
  Serial.println("\nCalibration Results:");
  Serial.print("  Zero offset: ");
  Serial.println(zeroOffset);
  Serial.print("  Calibration factor: ");
  Serial.println(calibrationFactor);
  
  // Test the calibration
  Serial.println("\nTesting calibration...");
  while (!scaleHandler.available()) delay(10);
  long testReading = scaleHandler.getReading();
  float testWeight = calibrationFactor * (testReading - zeroOffset);
  
  Serial.print("Test reading: ");
  Serial.println(testReading);
  Serial.print("Calculated weight: ");
  Serial.print(testWeight);
  Serial.println(" g (should be close to " + String(knownWeight) + " g)");
  
  // Save calibration data
  isCalibrated = true;
  saveCalibrationData();
  
  Serial.println("Calibration complete and saved to EEPROM.");
}

// Save calibration data to EEPROM
void saveCalibrationData() {
  // Flag as calibrated
  EEPROM.update(EEPROM_ADDR_CALIBRATED, (byte)1);
  
  // Save zero offset
  EEPROM.put(EEPROM_ADDR_ZERO_OFFSET, zeroOffset);
  
  // Save calibration factor
  EEPROM.put(EEPROM_ADDR_CALIB_FACTOR, calibrationFactor);
  
  Serial.println("Calibration data saved to EEPROM:");
  Serial.print("  Zero offset: ");
  Serial.println(zeroOffset);
  Serial.print("  Calibration factor: ");
  Serial.println(calibrationFactor);
}

// Load calibration data from EEPROM
void loadCalibrationData() {
  Serial.println("Loading calibration data from EEPROM...");
  
  byte calibrated = EEPROM.read(EEPROM_ADDR_CALIBRATED);
  
  if (calibrated == 1) {
    // Read calibration values
    EEPROM.get(EEPROM_ADDR_ZERO_OFFSET, zeroOffset);
    EEPROM.get(EEPROM_ADDR_CALIB_FACTOR, calibrationFactor);
    
    // Validate values
    if (calibrationFactor <= 0 || isnan(calibrationFactor) || isinf(calibrationFactor)) {
      Serial.println("Invalid calibration factor in EEPROM. Using defaults.");
      zeroOffset = 0;
      calibrationFactor = 1.0;
      isCalibrated = false;
    } else {
      isCalibrated = true;
      Serial.println("Calibration data loaded:");
      Serial.print("  Zero offset: ");
      Serial.println(zeroOffset);
      Serial.print("  Calibration factor: ");
      Serial.println(calibrationFactor);
    }
  } else {
    Serial.println("No calibration data found in EEPROM.");
    zeroOffset = 0;
    calibrationFactor = 1.0;
    isCalibrated = false;
  }
}

// Start recording data to SD card
void startRecording() {
  Serial.println("\nStarting recording...");
  isRecording = true;
  
  // Create a new filename
  createFilename();
  
  // Open the file
  dataFile = SD.open(filename.c_str(), FILE_WRITE);
  
  if (dataFile) {
    Serial.print("Recording to: ");
    Serial.println(filename);
    
    // Reset buffer
    bufferPos = 0;
    
    // Initialize thermocouple if it's working
    if (thermocoupleWorking) {
      thermocouple.begin(MCP9601_I2C_ADDR);
      Serial.println("Thermocouple initialized for recording");
    }
    
    // Write header
    addToBuffer("Time(us),Thrust(g),Temperature(C),Pressure(PSI)\r\n");
    
    // Fast LED blink to indicate recording started
    for (int i = 0; i < 3; i++) {
      analogWrite(LED_PIN, LED_BRIGHTNESS_ERROR);  // Bright flash
      delay(50);
      analogWrite(LED_PIN, 0);
      delay(50);
    }
    
    // Set LED to recording brightness
    analogWrite(LED_PIN, LED_BRIGHTNESS_ACTIVE);
    
    lastFlushTime = millis();
  } else {
    Serial.println("Error opening file!");
    isRecording = false;
    errorBlink(5);
  }
}

// Stop recording data
void stopRecording() {
  Serial.println("\nStopping recording...");
  isRecording = false;
  
  // Flush remaining data
  flushBuffer();
  
  // Close file
  if (dataFile) {
    dataFile.close();
    Serial.println("File closed: " + filename);
  }
  
  // Return to standby LED brightness
  analogWrite(LED_PIN, LED_BRIGHTNESS_STANDBY);
}

// Collect and log data from all sensors
void collectAndLogData() {
  // Get timestamp
  unsigned long timestamp = micros();
  
  // Get weight using calibrated values
  float thrust = 0.0;
  if (scaleHandler.available()) {
    long rawReading = scaleHandler.getReading();
    thrust = calibrationFactor * (rawReading - zeroOffset);
  }
  
  // Get temperature from MCP9601 if it's working
  float temperature = -999.99; // Default value if not working
  if (thermocoupleWorking) {
    // Read directly without reinitializing
    temperature = thermocouple.readThermocouple();
  }
  
  // Get pressure from transducer with correct offset
  int rawPressure = analogRead(PRESSURE_PIN);
  float voltage = (rawPressure * 3.3) / 4095.0;
  float pressure = calculatePressure(voltage);
  
  // Create data string and add to buffer
  char dataString[100]; // Make sure this is large enough
  snprintf(dataString, sizeof(dataString), 
          "%lu,%.2f,%.2f,%.2f\r\n", 
          timestamp, thrust, temperature, pressure);
  
  // Add to buffer
  addToBuffer(dataString);
}

// Add data to buffer
void addToBuffer(const char* data) {
  // Add data to buffer
  int dataLen = strlen(data);
  
  // Check if buffer is almost full
  if (bufferPos + dataLen >= BUFFER_SIZE - 1) {
    flushBuffer(); // Flush buffer if it would overflow
  }
  
  // Copy data to buffer
  strcpy(&dataBuffer[bufferPos], data);
  bufferPos += dataLen;
}

// Flush buffer to SD card
void flushBuffer() {
  if (bufferPos > 0 && dataFile) {
    // Null terminate the buffer
    dataBuffer[bufferPos] = '\0';
    
    // Write the entire buffer to SD card at once
    dataFile.write(dataBuffer, bufferPos);
    
    // Reset buffer position
    bufferPos = 0;
  }
}

// Create a unique filename for data
void createFilename() {
  // Create a unique filename based on millis() since boot
  filename = "DATA_";
  filename += String(millis());
  filename += ".csv";
}

// Error blink pattern
void errorBlink(int errorCode) {
  // Blink LED to indicate error codes
  while (1) {
    for (int i = 0; i < errorCode; i++) {
      analogWrite(LED_PIN, LED_BRIGHTNESS_ERROR);
      delay(200);
      analogWrite(LED_PIN, 0);
      delay(200);
    }
    delay(1000);
  }
}

void testThermocouple() {
  Serial.println("\n=== THERMOCOUPLE TEST ===");
  
  if (!thermocouple.begin(MCP9601_I2C_ADDR)) {
    Serial.println("ERROR: Thermocouple not initialized!");
    Serial.println("The MCP9601 was not detected during startup.");
    return;
  }


  Serial.println("Taking thermocouple readings (press any key to stop):");
  Serial.println("Hot Junction (°C) | Cold Junction (°C)");
  Serial.println("--------------------|------------------");  
      // Hot junction temperature (main temperature reading)
  float hotTemp = thermocouple.readThermocouple();
      
      // Cold junction (ambient) temperature
  float coldTemp = thermocouple.readAmbient();
  Serial.println(hotTemp);
  Serial.println(coldTemp);
  while (Serial.available()) Serial.read();
  
  Serial.println("Thermocouple test complete.");
}
