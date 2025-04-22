#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include "memorysaver.h"

const int CS = 10;
const int CAM_POWER_ON = 0;


ArduCAM myCAM(OV5642, CS);


void initializeCamera();
void checkCameraModule();
void startCapture();
void waitForCaptureComplete();
void sendCapturedImageOverSerial();

void setup() {
  Serial.begin(115200);
  pinMode(CS, OUTPUT);
  pinMode(CAM_POWER_ON , OUTPUT);
  digitalWrite(CAM_POWER_ON, HIGH);

  Wire.begin();
  SPI.begin();
  SPI.setFrequency(4000000); // 4MHz

  Serial.println(F("[INFO] ArduCAM Serial Image Capture"));

  initializeCamera();
  checkCameraModule();

  myCAM.set_format(JPEG);
  myCAM.InitCAM();

  myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
  myCAM.OV5642_set_JPEG_size(OV5642_1280x960);


  myCAM.clear_fifo_flag();
  delay(1000);
}

void loop() {
  startCapture();
  waitForCaptureComplete();
  sendCapturedImageOverSerial();
  delay(5000);  // Optional: Wait before next capture
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
    Serial.println(F("[ERROR] Can't find OV5642 module!"));
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

void sendCapturedImageOverSerial() {
  uint32_t len = myCAM.read_fifo_length();

  if (len == 0) {
    Serial.println(F("[ERROR] Captured image is empty."));
    return;
  }

  if (len >= MAX_FIFO_SIZE) {
    Serial.println(F("[ERROR] Image too large for FIFO."));
    return;
  }

  Serial.print(F("[INFO] Image size: "));
  Serial.print(len);
  Serial.println(" bytes");

  myCAM.CS_LOW();
  myCAM.set_fifo_burst();

  Serial.println("IMG_START");  // Marker for receiver

  while (len--) {
    uint8_t b = SPI.transfer(0x00);
    Serial.write(b);  // Send image byte
  }

  myCAM.CS_HIGH();
  myCAM.clear_fifo_flag();

  Serial.println("IMG_END");  // Marker for receiver
  Serial.println(F("[INFO] Image sent over Serial."));
}
