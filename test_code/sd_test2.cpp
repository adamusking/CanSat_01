#include <SPI.h>
#include <SdFat.h>

// Pin Definitions (adjust based on your wiring)
#define SD_CS 35 // Modify based on your setup

#define SCK_PIN  40
#define MISO_PIN 42
#define MOSI_PIN 36

// SdFat object
SdFat SD;

// SdFile object to handle file operations
SdFile myFile;

// Setup function
void setup() {
  Serial.begin(115200);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  // Initialize SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");
  
  // Open the CSV file (create it if it doesn't exist)
  if (!myFile.open("data.csv", O_WRITE | O_CREAT | O_APPEND)) {
    Serial.println("Failed to open file.");
    return;
  }
  
  // Optional: Write header to CSV file (only once)
  myFile.println("Timestamp, Temperature (C), Gas Concentration (ppm), Coordinates (Lat, Lon), Altitude (m)");
  
  // Close the file initially to start fresh writing
  myFile.close();
}

// Loop function
void loop() {
  // Random values (replace with your sensor readings)
  float temperature = random(15, 30);  // Random temperature between 15 and 30
  float gasConcentration = random(100, 500);  // Random gas concentration between 100 and 500 ppm
  float latitude = random(3000, 4000) / 100.0;  // Random latitude between 30.00 and 40.00
  float longitude = random(10000, 20000) / 100.0;  // Random longitude between 100.00 and 200.00
  float altitude = random(100, 2000);  // Random altitude between 100m and 2000m
  
  // Open file for appending
  if (myFile.open("data.csv", O_WRITE | O_APPEND)) {
    // Write data to file (replace timestamp with actual timestamp generation logic)
    myFile.print("2025-05-01 12:34:56");  // Placeholder for timestamp
    myFile.print(", ");
    myFile.print(temperature);
    myFile.print(", ");
    myFile.print(gasConcentration);
    myFile.print(", ");
    myFile.print(latitude, 6);  // 6 decimal places for latitude
    myFile.print(", ");
    myFile.print(longitude, 6);  // 6 decimal places for longitude
    myFile.print(", ");
    myFile.println(altitude);
    
    // Sync to ensure data is written to SD card
    myFile.sync();
    
    // Close the file
    myFile.close();
    
    Serial.println("Data written to SD card.");
  } else {
    Serial.println("Failed to open file for appending.");
  }
  
  // Wait before writing next data (simulating delay from sensors)
  delay(1000);
}
