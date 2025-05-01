#include <Arduino.h>
#include <FS.h>
#include <SD.h>

void setup() {
  Serial.begin(115200);
  
  if (SD.begin()) {
    File file = SD.open("/test.txt", FILE_WRITE);
    if (file) {
      file.println("Hello from ESP32 S3!");
      file.close();
      Serial.println("Wrote to file successfully");
    } else {
      Serial.println("Failed to open file");
    }
  } else {
    Serial.println("Failed to initialize SD card");
  }
}

void loop() {
}
