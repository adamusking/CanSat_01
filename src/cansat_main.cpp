#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include "memorysaver.h"
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ADS1X15.h>
#include "Adafruit_SGP30.h"
#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <esp_system.h>



#define I2C_SDA 8
#define I2C_SCL 9

Adafruit_ADS1115 ads;
Adafruit_BMP3XX bmp;
TinyGPSPlus gps;  
Adafruit_SGP30 sgp;

HardwareSerial GPS(1);  // use UART1 for the GPS module

#define GPS_RX 16  //defining pins
#define GPS_TX 17 

const int CS = 10;
const int CAM_POWER_ON = 0;

const int pwmPin = 4;

ArduCAM myCAM(OV5642, CS);

float TVOC;
float temperature;
float latitude;
float longitude;
float pressure;
float altitude;
float co2ppm;
int camerainitialized;


void initializeCamera();
void checkCameraModule();
void startCapture();
void waitForCaptureComplete();
void sendCapturedImageOverSerial();
void readGPS();
void readSGP();
void startCapture();
void waitForCaptureComplete();
void sendCapturedImageOverSerial();
void bmpRead();
void s8Read();


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
    delay(100);

    pinMode(pwmPin, INPUT);

    Serial.println("Initializing BMP390");

    if (!bmp.begin_I2C(0x77)) {
        Serial.println("Could not find a valid BMP390 sensor, check wiring!");
    
    }
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

    GPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);  // start GPS UART communication at 9600 baud

    Serial.println("GPS Module Initialized. Waiting for GPS signal...");

    if (!sgp.begin()) {
      Serial.println("SGP30 not found! Check wiring.");
  }
  
  Serial.println("SGP30 Sensor Initialized");
  }

void loop(){
  bmpRead();
  readGPS();
  readSGP();
  startCapture();
  waitForCaptureComplete();
  sendCapturedImageOverSerial();
  s8Read();
  delay(500);
}


  void initializeCamera() {
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    uint8_t temp = myCAM.read_reg(ARDUCHIP_TEST1);
    if (temp != 0x55) {
      Serial.println(F("[ERROR] SPI interface Error!"));
      
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
      camerainitialized=1;
    }
  }

  void bmpRead() {
    if (!bmp.performReading()) {
        Serial.println("Failed to BMP read data!");
        return;
    }

    // print sensor data
    Serial.print("Temperature = ");
    Serial.print(bmp.temperature);
    temperature = bmp.temperature;
    Serial.println(" °C");

    Serial.print("Pressure = ");
    Serial.print(bmp.pressure / 100.0);
    temperature=(bmp.pressure/100);
    Serial.println(" hPa");

    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude(1013.25));
    altitude=bmp.readAltitude(1013.25);
    Serial.println(" m");
  }

  void readGPS() {
    while (GPS.available()) {  // check if GPS data is coming in
      gps.encode(GPS.read());  // process incoming characters
    }
    if (gps.location.isValid()) {  // ensure that we have a valid GPS fix
      Serial.print("Latitude: ");
      Serial.print(gps.location.lat(), 6);  // print latitude with 6 decimal places
      latitude=(gps.location.lat(), 6);
      Serial.print(" Longitude: ");
      Serial.print(gps.location.lng(), 6);  // print longitude with 6 decimal places
      longitude=(gps.location.lng(), 6);
      Serial.print(" Altitude: ");
      Serial.print(gps.altitude.meters());  // print altitude in meters
      Serial.print(" Speed: ");
      Serial.print(gps.speed.kmph());  // speed in km/h
      Serial.print(" Course: ");
      Serial.println(gps.course.deg());  // course in degrees
      
    
  }
  else{
    Serial.println("Invalid GPS data");
  }
  }
  void readSGP(){
    if (!sgp.IAQmeasure()) {
      Serial.println("SGP Measurement failed");
      return;
  }

  // Print CO2 and TVOC values
  Serial.print("CO2: ");
  Serial.print(sgp.eCO2);
  Serial.print(" ppm, TVOC: ");
  Serial.print(sgp.TVOC);
  TVOC=sgp.TVOC;
  Serial.println(" ppb");
  }
  void startCapture() {
    if (camerainitialized==1){
    Serial.println(F("[INFO] Starting capture..."));
    myCAM.clear_fifo_flag();
    myCAM.start_capture();
    }
  }
  
  void waitForCaptureComplete() {
    if (camerainitialized==1){
    while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
      delay(10);
    }
    Serial.println(F("[INFO] Capture complete."));
  }
}
  
  void sendCapturedImageOverSerial() {
    if (camerainitialized==1){
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
}

  void s8Read() {
    uint32_t highDuration = pulseIn(pwmPin, HIGH);
    uint32_t lowDuration = pulseIn(pwmPin, LOW);
    uint32_t period = highDuration + lowDuration;
    if (period > 0) {
        float co2Concentration = (highDuration / (float)period) * 2000.0;
        Serial.print("CO2 Concentration: ");
        Serial.print(co2Concentration);
        Serial.println(" ppm");
        co2ppm=co2Concentration;
        
    } else {
        Serial.println("Error: Invalid period CO2 measurement.");
        Serial.println(period);
    }
  }
  
