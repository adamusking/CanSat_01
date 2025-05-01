#include <Arduino.h>

const int buzzerPin = 8;

void setup() {
    pinMode(buzzerPin, OUTPUT);
}

void loop()
     {   
  tone(buzzerPin, 2000);    
  delay(500);

  noTone(buzzerPin);      
  delay(3000);  
     }