#include <Wire.h>
#include <SPI.h>
#include "ArduCAM.h"
#include "memorysaver.h"

// Define pins for ESP32 (change them as per your wiring)
#define CS_PIN  10   // Chip Select for ArduCam
#define SCK_PIN  12
 // SPI Clock
#define MISO_PIN 13  // SPI MISO
#define MOSI_PIN 11  // SPI MOSI

// Define I2C pins
#define SDA_PIN 8
#define SCL_PIN 9

ArduCAM myCAM(OV5642, CS_PIN);

void setup() {
    Serial.begin(115200);
    delay(1000);

    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);  

    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);  // Deactivate chip select initially
    
    Serial.println("Initializing ArduCAM OV5642...");

    myCAM.write_reg(0x07, 0x80);  // Reset the camera
    delay(100);
    myCAM.write_reg(0x07, 0x00);
    delay(100);
    
    Wire.begin(SDA_PIN, SCL_PIN);
    uint8_t vid, pid;
    myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
    
    if ((vid != 0x56) || (pid != 0x42)) {
        Serial.println("Camera initialization failed!");
        while (1);  // Stop here if initialization fails
    }

    Serial.println("Camera initialized successfully!");

    myCAM.set_format(JPEG);  // Set JPEG format
    myCAM.InitCAM();         // Initialize the camera module
    myCAM.OV5642_set_JPEG_size(OV5642_1024x768);  // Set the image resolution
    myCAM.clear_fifo_flag(); // Clear the FIFO flag
}

void loop() {
    Serial.println("Capturing image...");

    myCAM.flush_fifo();
    myCAM.clear_fifo_flag();
    myCAM.start_capture();
    
    while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));  // Wait for capture to finish
    Serial.println("Image captured!");

    uint32_t imageSize = myCAM.read_fifo_length();
    Serial.print("Image size: ");
    Serial.print(imageSize);
    Serial.println(" bytes");

    // Start sending data
    Serial.println("START_IMAGE");

    myCAM.CS_LOW();
    myCAM.set_fifo_burst();

    uint8_t byte;
    for (uint32_t i = 0; i < imageSize; i++) {
        byte = SPI.transfer(0x00);  // Use the ESP32 SPI interface
        Serial.print(byte, HEX);      // Print the byte in HEX format
        Serial.print(" ");
    }

    myCAM.CS_HIGH();
    Serial.println("\nEND_IMAGE");

    delay(5000);  // Wait before capturing another image
}
