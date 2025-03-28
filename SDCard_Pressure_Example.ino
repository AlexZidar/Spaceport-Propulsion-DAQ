#include <SPI.h>
#include <SD.h>

const int pressurePin = A0; // Analog input pin that the pressure transducer is attached to
const float Vmin = 0.5;     // Minimum voltage output from the transducer
const float Vmax = 4.5;     // Maximum voltage output from the transducer
const float PSImax = 2500.0; // Maximum pressure the transducer can read
const int chipSelect = 10;  // Chip select pin for the microSD card module

unsigned long startTime;    // Variable to store the start time

void setup() {
  Serial.begin(115200); // Initialize serial communication at 9600 baud

  // Initialize the SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("Initialization done.");

  startTime = millis(); // Record the time when the Arduino powers on
}

void loop() {
  unsigned long currentTime = millis(); // Get the current time

  // Check if 10 seconds have passed since the Arduino powered on
  if (currentTime - startTime <= 1000) {
    int sensorValue = analogRead(pressurePin); // Read the analog input (value between 0 and 1023)
    float voltage = sensorValue * (5.0 / 1023.0); // Convert the analog reading to a voltage

    // Convert the voltage to a pressure value
    float pressure = (voltage - Vmin) * (PSImax / (Vmax - Vmin));

    // Print the timestamp and pressure to the Serial Monitor
    Serial.print("Timestamp: ");
    Serial.print(currentTime); // Print the time in seconds
    Serial.print(" ms, Pressure: ");
    Serial.print(pressure);
    Serial.println(" PSI");

    // Open the file in write mode
    File dataFile = SD.open("datalog.txt", FILE_WRITE);

    // If the file is available, write to it
    if (dataFile) {
      dataFile.print("Timestamp: ");
      dataFile.print(currentTime); // Print the time in seconds
      dataFile.print(" ms, Pressure: ");
      dataFile.print(pressure);
      dataFile.println(" PSI");
      dataFile.close(); // Close the file
    } else {
      // If the file isn't open, print an error
      Serial.println("Error opening datalog.txt");
    }
  }
}
