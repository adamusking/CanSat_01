// this does not convert volts to ppm

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

void setup() {
  Serial.begin(115200);
  // initialize ADS1115
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS1115.");
    while (1);
  }
  ads.setGain(GAIN_ONE); // +/- 4.096V
}

void loop() {
 
  int16_t adc0 = ads.readADC_SingleEnded(0);

  float voltage = ads.computeVolts(adc0);
  // print the results
  Serial.print("ADC Value: "); Serial.print(adc0);
  Serial.print("\tVoltage: "); Serial.print(voltage, 4); Serial.println(" V");

  delay(1000); 
}
