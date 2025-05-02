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
#include <SdFat.h>



#define I2C_SDA 45
#define I2C_SCL 48

#define SD_CS 35
#define COM_CS 47

#define GPS_RX 6  //defining pins
#define GPS_TX 7

#define SCK_PIN  40
#define MISO_PIN 42
#define MOSI_PIN 36

const int CAM_CS = 1;
const int buzzerPin = 8;
const int pwmPin = 4;// 15 alebo 17

#define BATTERY_PIN 20            
#define ADC_MAX 4095              
#define ADC_REF_VOLTAGE 3.3         // Default ADC reference voltage
#define VOLTAGE_DIVIDER_SCALE 8.4 / 2.9178  
#define BMS_CUTOFF_VOLTAGE 6.0 

Adafruit_ADS1115 ads;
Adafruit_BMP3XX bmp;
TinyGPSPlus gps;  
Adafruit_SGP30 sgp;
SdFat SD;
SdFile myFile;
HardwareSerial GPS(1);  // use UART1 for the GPS module


ArduCAM myCAM(OV5642, CAM_CS);



float TVOC;
float temperature;
float latitude;
float longitude;
float speed;
float pressure;
float pressureAltitude;
float pressureAltSeaLevel;
float co2ppm;
float CO;
float NO2;
float SO2;
float CH4;
float gpsAltitudeSeaLevel;
float gpsAltSeaLevel;
float battery;
float measured_voltage;      // From ADC reading
float battery_voltage;
float startingPressure;
float gpsStartingAlt;
float altSum;
float gpsAltitude;

int camerainitialized;
const int maxRetries=5;

int attempt;
int raw_adc;
int counting;
int countingGPS;
int pressureSum;

char filename[32];

char timestamp[20]; 


void initializeCamera();
void checkCameraModule();
void startCapture();
void waitForCaptureComplete();
void sendCapturedImageOverSerial();
void readGPS();
void readSGP();
void bmpRead();
void s8Read();
void saveSensorData();
void readAnalogSensors();
void writeDataToSD();
void saveImageToSD();
void baterryPercentage();
String getTimestamp();


void setup() {
  
    pinMode(buzzerPin, OUTPUT);
    Serial.begin(115200);
    
    pinMode(CAM_CS, OUTPUT);
    pinMode(buzzerPin, OUTPUT);
    pinMode(SD_CS, OUTPUT);
    pinMode(BATTERY_PIN, INPUT);
    analogReadResolution(12);

    Wire.begin(I2C_SDA, I2C_SCL);
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
    //SPI.setFrequency(4000000); // 4MHz
  
    /*Serial.println(F("[INFO] ArduCAM Serial Image Capture"));
  
    initializeCamera();
    checkCameraModule();
  
    myCAM.set_format(JPEG);
    myCAM.InitCAM();
  
    myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    myCAM.OV5642_set_JPEG_size(OV5642_1280x960);
  
  
    myCAM.clear_fifo_flag();
    delay(100);*/

    pinMode(pwmPin, INPUT);

    

    /*while (!SD.begin(SD_CS)&& attempt<maxRetries) {
      Serial.println("[ERROR] SD Card initialization failed!");
      Serial.print(attempt);
      attempt++;
    }
    if (attempt==maxRetries) {
      Serial.println("[INFO] SD Card failed to initialize");
      attempt=0;
    }
    else{
      Serial.println("[INFO] SD Card initialized successfully.");
      attempt=0;
    }*/

    Serial.println("Initializing BMP390");

    while (!bmp.begin_I2C(0x77) && attempt<maxRetries) {
        Serial.println("Could not find BMP390 sensor");
        Serial.print(attempt);
        attempt++;
        delay(50);
    }
    if (attempt==maxRetries){
        Serial.println("BMP Initialization failed, moving on");
        attempt=0;
    }
    else if (attempt<maxRetries) {
      Serial.println("BMP initialized successfully");
      attempt=0;
    }

    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

    GPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);  // start GPS UART communication at 9600 baud

    Serial.println("GPS Module Initialized. Waiting for GPS signal...");

    while (!sgp.begin()&& attempt<maxRetries) {
      Serial.println("SGP30 not found!");
      Serial.print(attempt);
      attempt++;
      delay(50);

  }
  if (attempt==maxRetries){
    Serial.println("SGP30 not found, moving on");
    attempt=0;
  }
  else {Serial.println("SGP30 Sensor Initialized");
  attempt=0;
  }

  while (!ads.begin()&&attempt<maxRetries) {
    Serial.println("Failed to initialize ADS1115!");
    Serial.print(attempt);
    attempt++;
  }
  if (attempt==maxRetries)
  {
    Serial.println("ADS not found, moving on");
    attempt=0;
  }
  else {
    Serial.println("ADS initialized");
    ads.setGain(GAIN_ONE); 
    }
    while (!SD.begin(SD_CS)&&attempt<maxRetries) {
      Serial.println("SD card initialization failed!");
      Serial.print(attempt);
      attempt++;
    }
    if (attempt=maxRetries)
      {
      Serial.println("SD card failed to initialize, moving on");
      attempt=0;
      }
    else{
    Serial.println("SD card initialized.");
    attempt=0;
    }
    if (!myFile.open("data.csv", O_WRITE | O_CREAT | O_APPEND)) {
      Serial.println("Failed to open file.");
      
    }
    myFile.println("Time, Temperature, CO (ppm), CO2 (ppm), TVOC (ppb), NO2 (ppm), SO2 (ppm),  CH4 (ppm), Coordinates (Lat, Lon), Altitude GPS (m), Pressure (hPa), Speed (km/h), Altitude pressure (m) ");
    myFile.close();
    }



void loop(){
  //bmpRead();
  readGPS();
  readSGP();

  //startCapture();
  //waitForCaptureComplete();
  //sendCapturedImageOverSerial();

  //s8Read();

  readAnalogSensors();
  writeDataToSD();
  
  //saveImageToSD();
  
  baterryPercentage();
  //tone(buzzerPin, 8000);  
  //delay(500);
  //noTone(buzzerPin);      
  String timestamp = getTimestamp();
  
  delay(1000);
}


  /*void initializeCamera() {
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    uint8_t temp = myCAM.read_reg(ARDUCHIP_TEST1);
    while ((temp != 0x55)&&attempt<maxRetries) {
      Serial.println(F("[ERROR] SPI interface Error!"));
      Serial.print(attempt);
      attempt++;
    }
    if (attempt==maxRetries){
      Serial.println("Camera SPI initialization failed, moving on");
      attempt=0;
    }
    else {Serial.println("SPI initialized correctly");
    attempt=0;
    }
  }
  
  void checkCameraModule() {
    uint8_t vid, pid;
  
  
    myCAM.wrSensorReg16_8(0xff, 0x01);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
    while (((vid != 0x56) || (pid != 0x42))&& attempt<maxRetries) {
      Serial.println(F("[ERROR] Can't find OV5642 module!"));
      Serial.print(attempt);
      attempt++;
    } 
    if (attempt==maxRetries){
      Serial.println("[INFO] Camera initialization failed, moving on");
      attempt=0;
    }
    else 
    {
      Serial.println(F("[INFO] OV5642 detected."));
      camerainitialized=1;
      attempt=0;
    }
  }*/

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
    pressure=(bmp.pressure/100);
    Serial.print(pressure);
    Serial.println(" hPa");

    Serial.print("Altitude from sea level = ");
    Serial.print(bmp.readAltitude(1013.25));
    pressureAltitude=bmp.readAltitude(1013.25);
    Serial.println(" m");

    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude(startingPressure));
    pressureAltitude=bmp.readAltitude(startingPressure);
    Serial.println(" m");

    if (counting < 10)
    {
      counting++;
      pressureSum=pressureSum+pressure;
    }
    else if (counting==10)
    {
      startingPressure=pressureSum/10;
      counting++;
    }
  }

  void readGPS() {
    while (GPS.available()) { 
      gps.encode(GPS.read());  // process incoming characters
    }
    if (gps.location.isValid()) {  
      Serial.print("Latitude: ");
      Serial.print(gps.location.lat(), 6);  
      latitude=(gps.location.lat(), 6);
      Serial.print(" Longitude: ");
      Serial.print(gps.location.lng(), 6);
      longitude=(gps.location.lng(), 6);
      Serial.print(" Altitude: ");
      Serial.print(gps.altitude.meters());
      gpsAltitudeSeaLevel=gps.altitude.meters();
      if (countingGPS<5)
      {
        countingGPS++;
        altSum=gpsAltitudeSeaLevel+altSum;
      }
      else if (countingGPS==5)
      {
        countingGPS++;
        gpsStartingAlt=altSum/5;
      }
      gpsAltitude=gpsAltitudeSeaLevel-gpsStartingAlt;

      Serial.print(" Speed: ");
      Serial.print(gps.speed.kmph());
      speed=gps.speed.kmph();
      Serial.print(" Course: ");
      Serial.println(gps.course.deg());
      
     // gpsAvailable = gps.location.isValid() && gps.time.isValid();
     const char* timestamp = "20240501_153210";
    char filename[32];
    snprintf(filename, sizeof(filename), "%s.jpg", timestamp);
    
  }
  else{
    Serial.println("Invalid GPS data");
    //gpsAvailable = gps.location.isValid() && gps.time.isValid();
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
 /* void startCapture() {
    if (camerainitialized==1){
    Serial.println(F("[INFO] Starting capture..."));
    myCAM.clear_fifo_flag();
    myCAM.start_capture();
    }
  }*/
  
 /* void waitForCaptureComplete() {
    if (camerainitialized==1){
    while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
      delay(10);
    }
    Serial.println(F("[INFO] Capture complete."));
  }
}*/
  
 /*void sendCapturedImageOverSerial() {
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
  
    /*while (len--) {
      uint8_t b = SPI.transfer(0x00);
      Serial.write(b);  // Send image byte
    }*/
  
    /*myCAM.CS_HIGH();
    myCAM.clear_fifo_flag();
  
    Serial.println("IMG_END");  // Marker for receiver
    Serial.println(F("[INFO] Image sent over Serial."));
  }
}*/

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
  void readAnalogSensors() {
    int16_t adc0 = ads.readADC_SingleEnded(0);
    float voltage = adc0 * 0.000125;  // convert ADC value to voltage
    
    float concentration=voltage/0.02;
  
    CO=concentration;
    Serial.print("CO Concentration: ");
    Serial.print(concentration);
    Serial.print(" ppm ");
    Serial.println(adc0);

    adc0 = ads.readADC_SingleEnded(1);
    voltage = adc0 * 0.000125;  // convert ADC value to voltage
  
    concentration=voltage/0.02;

    SO2=concentration;
    Serial.print("SO2 Concentration: ");
    Serial.print(concentration);
    Serial.print(" ppm ");
    Serial.println(adc0);

    adc0 = ads.readADC_SingleEnded(2);
    voltage = adc0 * 0.000125;  // convert ADC value to voltage
  
    concentration=voltage/0.03;
    
    NO2=concentration;
    Serial.print("NO2 Concentration: ");
    Serial.print(concentration);
    Serial.print(" ppm ");
    Serial.println(adc0);

    adc0 = ads.readADC_SingleEnded(3);
    Serial.print("CH4 ADC value: ");
    Serial.println(adc0);
  }
void writeDataToSD() {
  if (myFile.open("data.csv", O_WRITE | O_APPEND)) {
    // Write data to file (replace timestamp with actual timestamp generation logic)
    myFile.print(timestamp);  // Placeholder for timestamp
    myFile.print(", ");
    myFile.print(temperature);
    myFile.print(", ");
    Serial.println(NO2);
    myFile.print(", ");
    myFile.print(CO);
    myFile.print(", ");
    myFile.print(NO2);
    myFile.print(", ");
    myFile.print(SO2);
    myFile.print(", ");
    myFile.print(TVOC);
    myFile.print(", ");
    myFile.print(CH4);
    myFile.print(", ");
    myFile.print(latitude, 6);  // 6 decimal places for latitude
    myFile.print(", ");
    myFile.print(longitude, 6);  // 6 decimal places for longitude
    myFile.print(", ");
    myFile.println(pressureAltitude);
    myFile.print(gpsAltitude);
    myFile.print(", ");
    myFile.print(CH4);
    myFile.print(", ");
    
    // Sync to ensure data is written to SD card
    myFile.sync();
    
    // Close the file
    myFile.close();
    
    Serial.println("Data written to SD card.");
  } else {
    Serial.println("Failed to open file for appending.");
  }
  
}

/*void saveImageToSD() {
  String timestamp = getTimestamp();  // Call your timestamp function
  char filename[32];
  snprintf(filename, sizeof(filename), "%s.jpg", timestamp.c_str());

  if (!myFile.open(filename, O_WRITE | O_CREAT | O_TRUNC)) {
    Serial.println("File open failed");
    return;
  }

  myCAM.CS_LOW();
  SPI.transfer(BURST_FIFO_READ);

  const uint8_t BURST_SIZE = 128;
  uint8_t buf[BURST_SIZE];
  uint32_t remaining = len;  // 'len' must be the number of bytes in the image

  while (remaining > 0) {
    uint8_t toRead = remaining >= BURST_SIZE ? BURST_SIZE : remaining;
    for (uint8_t i = 0; i < toRead; i++) {
      buf[i] = SPI.transfer(0x00);
    }
    myFile.write(buf, toRead);
    remaining -= toRead;
  }

  myFile.close();  // Don't forget to close the file!
  myCAM.CS_HIGH();

  Serial.print("Saved image as: ");
  Serial.println(filename);
}
*/

void baterryPercentage()
{
  raw_adc = analogRead(BATTERY_PIN);
  measured_voltage = (raw_adc / float(ADC_MAX)) * ADC_REF_VOLTAGE;      // From ADC reading
  battery_voltage = measured_voltage * VOLTAGE_DIVIDER_SCALE;           // Remap to battery voltage

  battery = 0.0;

  // Estimate battery percentage linearly (modify for more accuracy with real SOC curves)
  if (battery_voltage <= BMS_CUTOFF_VOLTAGE) {
    battery = 0.0;
  } else if (battery_voltage >= 8.4) {
    battery = 100.0;
  } else {
    battery = ((battery_voltage - BMS_CUTOFF_VOLTAGE) / (8.4 - BMS_CUTOFF_VOLTAGE)) * 100.0;
  }

  Serial.print("ADC Value: ");
  Serial.print(raw_adc);
  Serial.print(" | Voltage: ");
  Serial.print(battery_voltage, 2);
  Serial.print(" V | Battery: ");
  Serial.print(battery, 1);
  Serial.println(" %");

}

String getTimestamp() {
  if (gps.date.isValid() && gps.time.isValid()) {
    int year   = gps.date.year();
    int month  = gps.date.month();
    int day    = gps.date.day();
    int hour   = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();

    snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d-%02d-%02d-%02d",
             year, month, day, hour, minute, second);
    return String(timestamp);
  } else {
    return "0000-00-00-00-00-00"; // Invalid or no fix yet
  }
  
}