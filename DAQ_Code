// Include necessary libraries
#include <Wire.h>
#include <EEPROM.h>  // Needed to record user settings
#include <SPI.h>
#include <SD.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"  // Click here to get the library: http://librarymanager/All#SparkFun_NAU8702

// Create instance of the NAU7802 class
NAU7802 myScale;

// EEPROM locations to store 4-byte variables
#define EEPROM_SIZE 100                // Allocate 100 bytes of EEPROM
#define LOCATION_CALIBRATION_FACTOR 0  // Float, requires 4 bytes of EEPROM
#define LOCATION_ZERO_OFFSET 10        // Must be more than 4 away from the previous spot. int32_t, requires 4 bytes of EEPROM

#define PRESSURE_SENSOR_PIN A0  // Pin for pressure transducer
#define CS_PIN 10               // Chip select for SD card
#define TOGGLE_SWITCH_PIN 7     // Pin for toggle switch

bool settingsDetected = false;  // Used to prompt user to calibrate their scale
File dataFile;                  // Declare dataFile globally

void setup() {
  Serial.begin(115200);
  Serial.println("Qwiic Scale Example with Pressure Sensor");

  // Initialize I2C and set clock
  Wire1.begin();
  Wire1.setClock(400000);  // Qwiic Scale is capable of running at 400kHz if desired

  if (!myScale.begin(Wire1)) { // MUST initialize at wire1 for qwiic port
    Serial.println("Scale not detected. Please check wiring. Freezing...");
    while (true)
      ;
  }
  Serial.println("Scale detected!");

  readSystemSettings();  // Load zeroOffset and calibrationFactor from EEPROM
  myScale.setGain(NAU7802_GAIN_16);
  myScale.setSampleRate(NAU7802_SPS_320);  // Increase to max sample rate
  myScale.calibrateAFE();                  // Re-cal analog front end when we change gain, sample rate, or channel

  // Print initial settings
  Serial.print("Zero offset: ");
  Serial.println(myScale.getZeroOffset());
  Serial.print("Calibration factor: ");
  Serial.println(myScale.getCalibrationFactor());

  // Configure 10-bit ADC for pressure sensor
  analogReadResolution(10);  // Set resolution to 10 bits

  // Initialize SD card
  if (!SD.begin(CS_PIN)) {
    Serial.println("SD card initialization failed. Check connections.");
    while (true)
      ;
  }
  Serial.println("SD card initialized.");

  // Log header to SD card
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Timestamp(ms),Weight(lbs),Pressure(PSI)");
    dataFile.close();
  }

  // Initialize toggle switch pin
  pinMode(TOGGLE_SWITCH_PIN, INPUT);
}

void loop() {
  if (myScale.available()) {
    float currentWeight = myScale.getWeight(true, 1, 0);

    // Read pressure transducer
    int pressureRaw = analogRead(PRESSURE_SENSOR_PIN);
    float pressureVoltage = pressureRaw * (5.0 / 1024);  // Convert raw ADC value to voltage
    float pressurePSI = mapPressureToPSI(pressureVoltage);

    // Get current time
    unsigned long currentTime = millis();

    // Check switch state and log/display data accordingly
    if (digitalRead(TOGGLE_SWITCH_PIN) == HIGH) {
      logDataToSD(currentTime, currentWeight, pressurePSI);
    } else {
      // Close the dataFile if it is open
      if (dataFile) {
        dataFile.close();
      }
      // Display readings on Serial Monitor
      Serial.print(currentTime);
      Serial.print("\tWeight: ");
      Serial.print(currentWeight, 1);  // Print 1 decimal places
      Serial.print("\tPressure (PSI): ");
      Serial.print(pressurePSI, 1);
      Serial.println();
    }
      if (!settingsDetected) {
        Serial.print("\tScale not calibrated. Press 'c'.");
      }
  }

  if (Serial.available()) {
    char incoming = Serial.read();

    if (incoming == 't') {
      myScale.calculateZeroOffset();  // Tare the scale
    } else if (incoming == 'c') {
      calibrateScale();  // Calibrate
    }
  }
}

// Map voltage to PSI for a 0.5V to 4.5V range over 0 to 2500 PSI
float mapPressureToPSI(float voltage) {
  return (voltage - 0.5) * (2500.0 / 4.0);  // Linear mapping from voltage to PSI
}

// Log data to SD card
void logDataToSD(unsigned long time, float weight, float pressure) {
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(time);
    dataFile.print(",");
    dataFile.print(weight, 1);
    dataFile.print(",");
    dataFile.println(pressure, 1);
    dataFile.close();
  } else {
    Serial.println("Error opening datalog.txt");
  }
}

void readSystemSettings() {
  float settingCalibrationFactor;  // Value used to convert the load cell reading to lbs or kg
  int32_t settingZeroOffset;       // Zero value that is found when scale is tared

  // Retrieve calibration factor from EEPROM
  EEPROM.get(LOCATION_CALIBRATION_FACTOR, settingCalibrationFactor);
  if (settingCalibrationFactor == 0xFFFFFFFF) {
    settingCalibrationFactor = 1.0;  // Default to 1.0
    EEPROM.put(LOCATION_CALIBRATION_FACTOR, settingCalibrationFactor);
  }

  // Retrieve zero tare point from EEPROM
  EEPROM.get(LOCATION_ZERO_OFFSET, settingZeroOffset);
  if (settingZeroOffset == 0xFFFFFFFF) {
    settingZeroOffset = 0;  // Default to 0 - i.e., no offset
    EEPROM.put(LOCATION_ZERO_OFFSET, settingZeroOffset);
  }

  // Pass these values to the library
  myScale.setCalibrationFactor(settingCalibrationFactor);
  myScale.setZeroOffset(settingZeroOffset);

  settingsDetected = true;  // Assume for the moment that there are good cal values
  if (settingCalibrationFactor == 1.0 || settingZeroOffset == 0) {
    settingsDetected = false;  // Defaults detected. Prompt user to calibrate scale.
  }
}

void recordSystemSettings() {
  // Get various values from the library and commit them to NVM
  EEPROM.put(LOCATION_CALIBRATION_FACTOR, myScale.getCalibrationFactor());
  EEPROM.put(LOCATION_ZERO_OFFSET, myScale.getZeroOffset());
}

void calibrateScale() {
  Serial.println("\n\nScale calibration");

  Serial.println(F("Setup scale with no weight on it. Press a key when ready."));
  while (Serial.available()) Serial.read();   // Clear anything in RX buffer
  while (Serial.available() == 0) delay(10);  // Wait for user to press key

  myScale.calculateZeroOffset(64);  // Zero or Tare the scale. Average over 64 readings.
  Serial.print(F("New zero offset: "));
  Serial.println(myScale.getZeroOffset());

  Serial.println(F("Place known weight on scale. Press a key when weight is in place and stable."));
  while (Serial.available()) Serial.read();   // Clear anything in RX buffer
  while (Serial.available() == 0) delay(10);  // Wait for user to press key

  Serial.print(F("Please enter the weight, without units, currently sitting on the scale (for example '4.25'): "));
  while (Serial.available()) Serial.read();   // Clear anything in RX buffer
  while (Serial.available() == 0) delay(10);  // Wait for user to press key

  // Read user input
  float weightOnScale = Serial.parseFloat();
  Serial.println();

  myScale.calculateCalibrationFactor(weightOnScale, 64);  // Tell the library how much weight is currently on it
  Serial.print(F("New cal factor: "));
  Serial.println(myScale.getCalibrationFactor(), 2);

  Serial.print(F("New Scale Reading: "));
  Serial.println(myScale.getWeight(), 2);

  recordSystemSettings();  // Commit these values to EEPROM

  settingsDetected = true;
}
