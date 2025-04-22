#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include "memorysaver.h"

#if !(defined ESP32 )
#error Please select the ArduCAM ESP32 UNO board in the Tools/Board
#endif

#if !(defined (OV2640_MINI_2MP)||defined (OV5640_MINI_5MP_PLUS) || defined (OV5642_MINI_5MP_PLUS) \
    || defined (OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_BIT_ROTATION_FIXED) \
    ||(defined (ARDUCAM_SHIELD_V2) && (defined (OV2640_CAM) || defined (OV5640_CAM) || defined (OV5642_CAM))))
#error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif

const int CS = 10;
const int CAM_POWER_ON = 0;

#if defined (OV2640_MINI_2MP) || defined (OV2640_CAM)
  ArduCAM myCAM(OV2640, CS);
#elif defined (OV5640_MINI_5MP_PLUS) || defined (OV5640_CAM)
  ArduCAM myCAM(OV5640, CS);
#elif defined (OV5642_MINI_5MP_PLUS) || defined (OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_BIT_ROTATION_FIXED) ||(defined (OV5642_CAM))
  ArduCAM myCAM(OV5642, CS);
#endif

void start_capture() {
  Serial.println("[INFO] Starting capture...");
  myCAM.clear_fifo_flag();
  myCAM.start_capture();
}

void readCapturedImage() {
  uint32_t len = myCAM.read_fifo_length();

  if (len >= MAX_FIFO_SIZE) {
    Serial.println("[ERROR] Image too large for FIFO.");
    return;
  }

  if (len == 0) {
    Serial.println("[ERROR] Captured image is empty.");
    return;
  }

  Serial.print("[INFO] Image size: ");
  Serial.print(len);
  Serial.println(" bytes");

  myCAM.CS_LOW();
  myCAM.set_fifo_burst();

  Serial.println("IMG_START");  // Marker for start
  while (len--) {
    uint8_t b = SPI.transfer(0x00);
    Serial.write(b);  // Send raw image byte
  }
  myCAM.CS_HIGH();
  Serial.println("IMG_END");  // Marker for end

  Serial.println("[INFO] Image sent over Serial.");
}

void setup() {
  uint8_t vid, pid;
  uint8_t temp;

  pinMode(CS, OUTPUT);
  pinMode(CAM_POWER_ON , OUTPUT);
  digitalWrite(CAM_POWER_ON, HIGH);

  Wire.begin();
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("[INFO] ArduCAM Serial Image Capture"));

  SPI.begin();
  SPI.setFrequency(4000000); // 4MHz

  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  temp = myCAM.read_reg(ARDUCHIP_TEST1);
  if (temp != 0x55) {
    Serial.println(F("[ERROR] SPI interface Error!"));
    while (1);
  }

#if defined (OV2640_MINI_2MP) || defined (OV2640_CAM)
  myCAM.wrSensorReg8_8(0xff, 0x01);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
  if ((vid != 0x26 ) || (pid != 0x41 && pid != 0x42)) {
    Serial.println(F("[ERROR] Can't find OV2640 module!"));
  } else {
    Serial.println(F("[INFO] OV2640 detected."));
  }
#endif

  myCAM.set_format(JPEG);
  myCAM.InitCAM();

#if defined (OV2640_MINI_2MP) || defined (OV2640_CAM)
  myCAM.OV2640_set_JPEG_size(OV2640_320x240);
#endif

  myCAM.clear_fifo_flag();
  delay(1000);
}

void loop() {
  start_capture();

  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
    delay(10);
  }

  Serial.println("[INFO] Capture done.");
  readCapturedImage();

  delay(5000);  // Wait before next capture
}
