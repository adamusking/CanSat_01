#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Arduino.h>

const int buzzerPin = 8;
#define SDA 45
#define SCL 48

Adafruit_ADS1115 ads;


void setup() {
  pinMode(buzzerPin, OUTPUT);
  Serial.begin(115200);
  Wire.begin(SDA,SCL);

  if (!ads.begin(0x48)) {
    Serial.println("Failed to initialize ADS1115!");
    while (1);
    tone(buzzerPin, 2000);
    delay(1000);
    noTone(buzzerPin);
  }

  ads.setGain(GAIN_ONE);  // ±4.096V range
  tone(buzzerPin, 5000);
  delay(5000);
  noTone(buzzerPin);
}

void loop() {
  int16_t adc0 = ads.readADC_SingleEnded(0);
  if ((adc0<65536)&&(adc0>0))
  {
    tone(buzzerPin, 500);
    delay(500);
    noTone(buzzerPin);
  }
  float voltage = adc0 * 0.000125;  // convert ADC value to voltage
  Serial.println(adc0);
  float concentration=voltage/0.02;
  int counter;
  while (counter<concentration)
  {
    tone(buzzerPin, 2000);
    delay(100);
    noTone(buzzerPin);
    counter++;
  }
  Serial.print("CO Concentration: ");
  Serial.print(concentration);
  Serial.println(" ppm");
  counter=0;
  delay(1000);
}
