#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ArduCAM.h>
#include "memorysaver.h"

#define CS_PIN 10  // Chip Select pin connected to the camera

// SPI pins for ESP32-S3
#define SPI_SCK  12
#define SPI_MISO 13
#define SPI_MOSI 11

ArduCAM myCAM(OV5642, CS_PIN);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Arducam OV5642 Init");

  // Initialize dummy I2C on unused pins to prevent Wire errors
  Wire.begin(33, 32);  // Use any unused GPIOs

  // Initialize SPI bus
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CS_PIN);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  delay(100);

  // Reset Arducam via register
  myCAM.write_reg(0x07, 0x80);  // Software reset
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);

  // Verify communication with camera module
  uint8_t vid, pid;
  myCAM.wrSensorReg8_8(0xFF, 0x01);  // Select sensor register bank
  delay(10);
  myCAM.rdSensorReg8_8(OV5642_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg8_8(OV5642_CHIPID_LOW, &pid);

  Serial.printf("VID: 0x%02X, PID: 0x%02X\n", vid, pid);

  if ((vid != 0x56) || (pid != 0x42)) {
    Serial.println("Camera not detected!");
    while (true);
  } else {
    Serial.println("Camera detected.");
  }

  // Initialize camera settings
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.OV5642_set_JPEG_size(OV5642_640x480);  // Set resolution
  delay(1000);
}

void loop() {
  Serial.println("Capturing...");

  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.start_capture();

  // Wait for capture to complete
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));

  Serial.println("Capture done!");

  // Read the length of image data
  uint32_t len = myCAM.read_fifo_length();
  Serial.printf("Image length: %lu bytes\n", len);

  if (len >= MAX_FIFO_SIZE) {
    Serial.println("Image too large for buffer!");
    return;
  }

  if (len == 0) {
    Serial.println("No image captured.");
    return;
  }

  // Read and send image over Serial
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();

  while (len--) {
    uint8_t c = SPI.transfer(0x00);
    Serial.write(c);  // Send JPEG byte over Serial
  }

  myCAM.CS_HIGH();

  Serial.println("\nImage sent over Serial.");

  delay(5000);  // Wait before next capture
}
