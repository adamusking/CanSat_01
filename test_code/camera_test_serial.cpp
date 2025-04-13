#include <Wire.h>
#include <SPI.h>
#include "ArduCAM.h"
#include "memorysaver.h"

// Define SPI pins
#define CS_PIN  10   // Chip Select for ArduCam
#define SCK_PIN  12
 // SPI Clock
#define MISO_PIN 13  // SPI MISO
#define MOSI_PIN 11  // SPI MOSI

// Define I2C pins
#define SDA_PIN 8
#define SCL_PIN 9

ArduCAM myCAM(OV5642, CS_PIN);

void resetCamera() {
    Serial.println("Resetting Camera...");
    myCAM.write_reg(0x07, 0x80);  // Reset ArduCAM
    delay(100);
    myCAM.write_reg(0x07, 0x00);
    delay(100);
    myCAM.wrSensorReg16_8(0x3008, 0x82);  // Reset OV5642 sensor
    delay(100);

    // Force reinitialize settings
    myCAM.set_format(JPEG);
    myCAM.InitCAM();
    myCAM.OV5642_set_JPEG_size(OV5642_320x240);
    delay(1000);
}

void setup() {
    Serial.begin(115200);
    
    // Initialize I2C
    
    
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.println("Checking OV5642 at 0x78 and 0x36...");

    Wire.beginTransmission(0x78);
    if (Wire.endTransmission() == 0) {
        Serial.println("OV5642 detected at 0x78");
    } else {
        Serial.println("OV5642 NOT found at 0x78");
    }

    Wire.beginTransmission(0x36);
    if (Wire.endTransmission() == 0) {
        Serial.println("OV5642 detected at 0x36");
    } else {
        Serial.println("OV5642 NOT found at 0x36");
    }

    // Initialize SPI
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
    SPI.setFrequency(4000000);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);

    
    for (int i = 0; i < 10; i++) {
        uint8_t testReg = SPI.transfer(0x00);  // Dummy write/read to test SPI
        Serial.print("SPI test register value: 0x");
        Serial.println(testReg, HEX);
        delay(500);
    }
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
    
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    delay(100);
    digitalWrite(CS_PIN, LOW);
    delay(100);
    digitalWrite(CS_PIN, HIGH);
    delay(100);

    myCAM.write_reg(0x07, 0x80);  // Reset ArduCAM
    delay(100);
    myCAM.write_reg(0x07, 0x00);  
    delay(100);

    myCAM.wrSensorReg16_8(0x3008, 0x82);  // Reset OV5642
    delay(100);
    myCAM.set_format(JPEG);
    myCAM.InitCAM();
    myCAM.wrSensorReg16_8(0x3818, 0x81);  // Force JPEG output
    myCAM.wrSensorReg16_8(0x3621, 0xA7);  // Improve image processing
    myCAM.OV5642_set_JPEG_size(OV5642_640x480);
    delay(1000);
}

void loop() {

    resetCamera();
    Serial.println("Capturing image...");
    
    myCAM.flush_fifo();      // Clear FIFO
    myCAM.clear_fifo_flag();

    Serial.println("Enabling FIFO...");
    myCAM.write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
    delay(100);

    myCAM.set_format(JPEG);
    myCAM.InitCAM();
    myCAM.OV5642_set_JPEG_size(OV5642_320x240);
    delay(100);

    Serial.println("Starting capture...");
    myCAM.start_capture();
    delay(200);

    uint32_t fifoLen = myCAM.read_fifo_length();
    Serial.print("FIFO Length After Capture: ");
    Serial.println(fifoLen);

    myCAM.write_reg(ARDUCHIP_FRAMES, 0x00);
    myCAM.start_capture();   // Capture new image
    

    delay(200);  // Wait for capture to finish

    uint8_t trig = myCAM.read_reg(ARDUCHIP_TRIG);
    Serial.print("Trigger Register: 0x");
    Serial.println(trig, HEX);
    uint8_t fifoReg = myCAM.read_reg(ARDUCHIP_FIFO);
    Serial.print("FIFO Register: 0x");
    Serial.println(fifoReg, HEX);
    

    if (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
        Serial.println("[ERROR] Capture did not complete!");
        return;
    }
    Serial.println("Capture complete!");

    uint32_t length = myCAM.read_fifo_length();

    Serial.print("FIFO Length: ");
    Serial.println(length);

    uint8_t temp;
    myCAM.rdSensorReg16_8(0x300A, &temp);
    Serial.print("Sensor ID High Byte: ");
    Serial.println(temp, HEX);
    
    myCAM.rdSensorReg16_8(0x300B, &temp);
    Serial.print("Sensor ID Low Byte: ");
    Serial.println(temp, HEX);
    // Validate FIFO length
    if (length == 0 || length >= 0x7FFFFF) {
        Serial.println("[ERROR] Invalid image size!");
        return;
    }

    Serial.print("Image size: ");
    Serial.println(length);

    Serial.println("Reading FIFO manually...");
    
    myCAM.CS_LOW();
    SPI.transfer(0x3D);  // FIFO read command

    for (int i = 0; i < 20; i++) {  // Read first 20 bytes
        uint8_t data = SPI.transfer(0x00);
        Serial.print(data, HEX);
        Serial.print(" ");
    }

    myCAM.CS_HIGH();
    Serial.println("\nEnd of FIFO Read");

   /* if (length > 0) {
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
    }*/

    uint8_t fifoStatus = myCAM.read_reg(ARDUCHIP_FIFO);
    Serial.print("FIFO Status: 0x");
    Serial.println(fifoStatus, HEX);
    
    for (int i = 0; i < 10; i++) {
        uint8_t testReg = myCAM.read_reg(ARDUCHIP_TEST1);
        Serial.print("SPI Test Register: 0x");
        Serial.println(testReg, HEX);
        delay(200);
    }
    
    delay(5000);
}
