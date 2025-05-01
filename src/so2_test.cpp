#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Arduino.h>


#define SDA 45
#define SCL 48

Adafruit_ADS1115 ads;


void setup() {
 
  Serial.begin(115200);
  Wire.begin(SDA,SCL);

  if (!ads.begin(0x48)) {
    Serial.println("Failed to initialize ADS1115!");
    while (1);
   
  }

  ads.setGain(GAIN_ONE);  // ±4.096V range

}

void loop() {
  int16_t adc0 = ads.readADC_SingleEnded(0);
 
  float voltage = adc0 * 0.000125;  // convert ADC value to voltage
  Serial.println(adc0);
  float concentration=voltage/0.02;
  
  Serial.print("SO2 Concentration: ");
  Serial.print(concentration);
  Serial.println(" ppm");

  delay(1000);
}
