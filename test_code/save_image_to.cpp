#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include <SdFat.h>
#include "memorysaver.h"

// SD card SPI pins (custom SPI bus - HSPI)
#define SD_CS   35
#define SD_SCK  40
#define SD_MISO 42
#define SD_MOSI 36

#define CAM_CS 10
// I2C for ArduCAM
#define I2C_SDA 8
#define I2C_SCL 9

// SPI instances
  // second SPI bus for SD card

ArduCAM myCAM(OV5642, CAM_CS);
SdFat SD;
SdFile myFile;

uint16_t imageCounter = 1;
int attempt = 0;
const int maxRetries = 5;

void initializeCamera();
void checkCameraModule();
void startCapture();
void waitForCaptureComplete();
void saveCapturedImageToSD();

void setup() {
  Serial.begin(115200);

  pinMode(CAM_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(CAM_CS, HIGH);

  Wire.begin(I2C_SDA, I2C_SCL);
  
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  SPI.setFrequency(4000000);
  Serial.println(F("[INFO] Initializing ArduCAM..."));

  initializeCamera();
  checkCameraModule();

  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
  myCAM.OV5642_set_JPEG_size(OV5642_640x480);
  myCAM.clear_fifo_flag();

  Serial.println(F("[INFO] Initializing SD card..."));

  //SdSpiConfig sdConfig(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(25), &sdSPI);
  while (!SD.begin(SD_CS) && attempt < maxRetries) {
    Serial.print(F("[ERROR] SD init attempt "));
    Serial.println(attempt + 1);
    attempt++;
    delay(500);
  }

  if (attempt == maxRetries) {
    Serial.println(F("[FATAL] SD card failed to initialize."));
  } else {
    Serial.println(F("[INFO] SD card initialized."));
  }

  attempt = 0;
  delay(1000);
}

void loop() {
  startCapture();
  waitForCaptureComplete();
  saveCapturedImageToSD();
  delay(5000);
}

void initializeCamera() {
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  uint8_t temp = myCAM.read_reg(ARDUCHIP_TEST1);
  if (temp != 0x55) {
    Serial.println(F("[ERROR] SPI interface Error!"));
    while (1);
  }
}

void checkCameraModule() {
  uint8_t vid, pid;
  myCAM.wrSensorReg16_8(0xff, 0x01);
  myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
  if ((vid != 0x56) || (pid != 0x42)) {
    Serial.println(F("[ERROR] OV5642 not detected."));
  } else {
    Serial.println(F("[INFO] OV5642 detected."));
  }
}

void startCapture() {
  Serial.println(F("[INFO] Starting capture..."));
  myCAM.clear_fifo_flag();
  myCAM.start_capture();
}

void waitForCaptureComplete() {
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
    delay(10);
  }
  Serial.println(F("[INFO] Capture complete."));
}

void saveCapturedImageToSD() {
  uint32_t len = myCAM.read_fifo_length();
  if (len == 0 || len >= MAX_FIFO_SIZE) {
    Serial.println(F("[ERROR] Invalid image size."));
    return;
  }

  char filename[20];
  snprintf(filename, sizeof(filename), "/IMG_%05d.jpg", imageCounter++);

  if (!myFile.open(filename, O_CREAT | O_WRITE | O_TRUNC)) {
    Serial.println(F("[ERROR] File open failed."));
    return;
  }

  myCAM.CS_LOW();
  myCAM.set_fifo_burst();

  const size_t BURST_SIZE = 128;
  uint8_t buffer[BURST_SIZE];

  while (len > 0) {
    size_t toRead = (len >= BURST_SIZE) ? BURST_SIZE : len;
    for (size_t i = 0; i < toRead; i++) {
      buffer[i] = SPI.transfer(0x00);
    }
    myFile.write(buffer, toRead);
    len -= toRead;
  }

  myCAM.CS_HIGH();
  myCAM.clear_fifo_flag();
  myFile.close();

  Serial.print(F("[INFO] Saved image: "));
  Serial.println(filename);
}
