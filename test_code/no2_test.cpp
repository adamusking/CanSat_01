#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS1115!");
    while (1);
  }

  ads.setGain(GAIN_ONE);  // ±4.096V range
}

void loop() {
  int16_t adc0 = ads.readADC_SingleEnded(0);
  float voltage = adc0 * 0.000125;  // convert ADC value to voltage

  float load_resistor = 10.0;  // from the datasheet
  float sensitivity = 300e-9;    

  float current = voltage / load_resistor;  // calculate current
  float concentration = current / sensitivity;  // calculate CO concentration in ppm

  Serial.print("CO Concentration: ");
  Serial.print(concentration);
  Serial.println(" ppm");

  delay(1000);
}
