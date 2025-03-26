#include <Wire.h>
#include <SPI.h>
#include "ArduCAM.h"
#include "memorysaver.h"

// Define SPI pins
#define CS_PIN  10   // Chip Select for ArduCam
#define SCK_PIN 18   // SPI Clock
#define MISO_PIN 12  // SPI MISO
#define MOSI_PIN 11  // SPI MOSI

// Define I2C pins
#define SDA_PIN 8
#define SCL_PIN 9

ArduCAM myCAM(OV5642, CS_PIN);

void setup() {
    Serial.begin(115200);
    
    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    
    // Initialize SPI
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
    
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    
    Serial.println("Initializing camera...");
    myCAM.write_reg(0x07, 0x80);
    delay(100);
    myCAM.write_reg(0x07, 0x00);
    delay(100);
    
    uint8_t vid, pid;
    myCAM.wrSensorReg16_8(0xff, 0x01);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
    
    if ((vid != 0x56) || (pid != 0x42)) {
        Serial.println("[ERROR] Camera not detected!");
        while (1);
    }
    Serial.println("Camera initialized successfully!");
    
    myCAM.set_format(JPEG);
    myCAM.InitCAM();
    myCAM.OV5642_set_JPEG_size(OV5642_320x240);
    delay(1000);
}

void loop() {
    Serial.println("Capturing image...");
    
    myCAM.flush_fifo();      // Clear previous image
    myCAM.clear_fifo_flag();
    myCAM.start_capture();   // Capture new image
    
    while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
    Serial.println("Capture complete!");
    
    uint32_t length = myCAM.read_fifo_length();
    
    // Validate FIFO length
    if (length == 0 || length >= 0x7FFFFF) {
        Serial.println("[ERROR] Invalid image size!");
        return;
    }

    Serial.print("Image size: ");
    Serial.println(length);

    if (length > 0) {
        Serial.println("[STARTIMG]");  // Mark start of image data
        
        myCAM.CS_LOW();
        myCAM.set_fifo_burst();  // Burst mode for fast transfer

        uint8_t buffer[256];  // Reduced buffer size to avoid overflows
        while (length > 0) {
            uint8_t readLen = (length > 256) ? 256 : length;
            for (uint16_t i = 0; i < readLen; i++) {
                buffer[i] = SPI.transfer(0x00);
            }
            Serial.write(buffer, readLen);
            length -= readLen;
        }

        Serial.println("\n[ENDIMG]");  // Mark end of image data
        myCAM.CS_HIGH();
    }

    delay(5000);
}
