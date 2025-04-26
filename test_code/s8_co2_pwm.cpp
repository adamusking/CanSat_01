#include <Arduino.h>

const int pwmPin = 4;// GPIO pin connected to the Senseair S8 PWM output

void setup() {
    pinMode(pwmPin, INPUT);
    Serial.begin(115200);
}

void loop() {
    // Measure the duration of the HIGH pulse
    uint32_t highDuration = pulseIn(pwmPin, HIGH);
    
    // Measure the duration of the LOW pulse
    uint32_t lowDuration = pulseIn(pwmPin, LOW);
    
    // Calculate the total period of the PWM signal
    uint32_t period = highDuration + lowDuration;
    
    // Calculate CO2 concentration in ppm
    if (period > 0) {
        float co2Concentration = (highDuration / (float)period) * 2000.0;
        Serial.print("CO2 Concentration: ");
        Serial.print(co2Concentration);
        Serial.println(" ppm");
        Serial.println(period);
    } else {
        Serial.println("Error: Invalid period measurement.");
        Serial.println(period);
    }
    
    delay(2000); // Wait for 2 seconds before the next reading
}
