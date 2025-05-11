#include <Arduino.h>

// Function to compensate CH4 sensor readings (based on GM-402B datasheet)
float compensate_ch4(uint16_t adc_value) {
    // Sensor parameters (from GM-402B typical values)
    const float mv_per_ppm = 0.076f;        // Sensitivity (mV per ppm)
    const float baseline_mv = 33.0f;        // Baseline in clean air (approx. 0 ppm)
    const float adc_to_mv = 5000.0f / 65535.0f;  // 16-bit ADC (0-5V range)

    // Convert ADC to mV
    float sensor_v = adc_value * 0.000125;

    // Calculate ppm
    float ppm = (sensor_v - baseline_mv) / mv_per_ppm;
    if (ppm < 0.0f) ppm = 0.0f;  // Clamp to 0 for negative values

    return ppm;
}

void setup() {
    Serial.begin(115200);
    Serial.println("CH4 Sensor Test");
}

void loop() {
    uint16_t raw_adc = analogRead(A0);  // Read from the sensor (e.g., A0)
    float ppm = compensate_ch4(raw_adc);
    Serial.print("Raw ADC: ");
    Serial.print(raw_adc);
    Serial.print(" | CH4: ");
    Serial.print(ppm);
    Serial.println(" ppm");
    delay(1000);  // Update every second
}