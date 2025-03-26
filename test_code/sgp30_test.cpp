#include <Wire.h>
#include "Adafruit_SGP30.h"

// Define I2C Pins
#define SDA_PIN 8  // Change as per wiring
#define SCL_PIN 9  // Change as per wiring

Adafruit_SGP30 sgp;

void setup() {
    Serial.begin(115200);
    
    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);

    // Initialize SGP30
    if (!sgp.begin()) {
        Serial.println("SGP30 not found! Check wiring.");
        while (1);
    }
    
    Serial.println("SGP30 Sensor Initialized");
}

void loop() {
    if (!sgp.IAQmeasure()) {
        Serial.println("Measurement failed");
        return;
    }

    // Print CO2 and TVOC values
    Serial.print("CO2: ");
    Serial.print(sgp.eCO2);
    Serial.print(" ppm, TVOC: ");
    Serial.print(sgp.TVOC);
    Serial.println(" ppb");

    delay(1000);
}
