#include <Wire.h>
#include <Adafruit_BMP3XX.h>

// define I2C pins
#define I2C_SDA 8
#define I2C_SCL 9

Adafruit_BMP3XX bmp;

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing BMP390");

    // initialize I2C communication
    Wire.begin(I2C_SDA, I2C_SCL);

    // check if sensor is detected
    if (!bmp.begin_I2C()) {
        Serial.println("Could not find a valid BMP390 sensor, check wiring!");
        while (1);
    }

    // Set the sensor parameters
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
}

void loop() {
    if (!bmp.performReading()) {
        Serial.println("Failed to read data!");
        return;
    }

    // print sensor data
    Serial.print("Temperature = ");
    Serial.print(bmp.temperature);
    Serial.println(" °C");

    Serial.print("Pressure = ");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude(1013.25)); // adjust sea level pressure as needed
    Serial.println(" m");

    Serial.println("-------------------------");
    delay(1000);
}
