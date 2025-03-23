#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "ArduCAM.h"
#include "memorysaver.h"

#define OV5642_CAM

#define CAM_CS_PIN    10
#define SD_CS_PIN     14
#define SPI_MOSI_PIN  11
#define SPI_MISO_PIN  13
#define SPI_SCK_PIN   12
#define I2C_SDA_PIN   8
#define I2C_SCL_PIN   9

ArduCAM myCAM(OV5642, CAM_CS_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, CAM_CS_PIN);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  pinMode(CAM_CS_PIN, OUTPUT);
  digitalWrite(CAM_CS_PIN, HIGH);
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
  myCAM.OV5642_set_JPEG_size(OV5642_1024x768);
  delay(1000);

  pinMode(SD_CS_PIN, OUTPUT);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card initialization failed!");
    return;
  }
  Serial.println("SD Card initialized");
}

void loop() {
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.start_capture();
  Serial.println("Capturing image...");
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  Serial.println("Image captured");

  File imgFile = SD.open("/image.jpg", FILE_WRITE);
  if (!imgFile) {
    Serial.println("Failed to open file on SD card");
    return;
  }

  uint8_t temp;
  uint32_t length = myCAM.read_fifo_length();
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  while (length--) {
    temp = SPI.transfer(0x00);
    imgFile.write(temp);
  }
  myCAM.CS_HIGH();

  imgFile.close();
  Serial.println("Image saved to SD card");

  delay(5000);
}
