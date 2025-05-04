#include <Wire.h>
#include <SPI.h>
#include <ArduCAM.h>
#include "memorysaver.h"
#undef swap
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ADS1X15.h>
#include "Adafruit_SGP30.h"
#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <esp_system.h>
#include <SdFat.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <ESP32_FTPClient.h>

#define ss 47
#define rst 41
#define dio0 19

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
const int pwmPin = 3;

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

String timestamp;

#define WIFI_SSID "Unicron"
#define WIFI_PASSWORD "Unicron1234rq-"

char ftp_server[] = "192.168.0.1";
char ftp_user[]   = "anonymous";
char ftp_pass[]   = "heslo";
ESP32_FTPClient ftp (ftp_server,ftp_user,ftp_pass, 5000, 2);

int count = 0;
SX1276 radio = new Module(ss, dio0, rst);

// Flag for receiving
volatile bool receivedFlag = false;

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  receivedFlag = true;
}


float TVOC;
float temperature;
float latitude;
float longitude;
float speed;
float pressure;
float pressureAltitude;
float pressureAltSeaLevel;
float CO2;
float CO;
float NO2;
float SO2;
float CH4;
float gpsAltitudeSeaLevel;
float gpsAltSeaLevel;
float battery;
float measured_voltage;
float battery_voltage;
float startingPressure;
float gpsStartingAlt;
float altSum;
float gpsAltitude;
float verticalSpeed; // placeholder
float predictedLongitude; // placeholder
float predictedLatitude; // placeholder

int camerainitialized;
const int maxRetries=5;

int attempt;
int raw_adc;
int counting;
int countingGPS;
int pressureSum;

char filename[32];

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
void handleCommand(String cmd);
void wifi_ftp_transfer();
void lora_com();

typedef struct {
    uint16_t packetID;
    uint16_t temperature;
    uint16_t pressure;
    uint16_t gpsAltitude;
    uint16_t pressureAltitude;
    uint16_t gpsAltSeaLevel;
    uint16_t pressureAltSeaLevel;
    uint16_t verticalSpeed;
    uint16_t horizontalSpeed;
    uint16_t battery;
    uint32_t latitude;
    uint32_t longitude;
    uint32_t predictedLongitude;
    uint32_t predictedLatitude;
    uint16_t CO2;
    uint16_t CO;
    uint16_t CH4;
    uint16_t NO2;
    uint16_t SO2;
    uint16_t TVOC;
} __attribute__((packed)) TelemetryPacket;


void setup() {
  
    Serial.begin(115200);
    
    pinMode(CAM_CS, OUTPUT);
    pinMode(buzzerPin, OUTPUT);
    pinMode(SD_CS, OUTPUT);
    pinMode(BATTERY_PIN, INPUT);
    analogReadResolution(12);

    Wire.begin(I2C_SDA, I2C_SCL);
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
    //SPI.setFrequency(4000000); // 4MHz
  
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
    if (attempt==maxRetries)
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
    else {
    myFile.println("time, temperature, pressure, gpsAltitude, pressureAltitude, gpsAltSeaLevel, pressureAltSeaLevel, verticalSpeed, horizontalSpeed, battery, latitude, longitude, predictedLongitude, predictedLatitude, CO2, CO, CH4, NO2, SO2, TVOC");
    myFile.sync();
    myFile.close();
    }

    Serial.print(F("[SX1276] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }

  }
  state = radio.setFrequency(869.525);
  state = radio.setBandwidth(125.0);
  state = radio.setSpreadingFactor(7);
  state = radio.setCodingRate(5);
  state = radio.setSyncWord(0xA5);
  state = radio.setOutputPower(17);
  state = radio.setPreambleLength(12);
  state = radio.setCRC(true); 

  radio.setPacketReceivedAction(setFlag);

  Serial.print(F("[SX1276] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
  }


void loop(){
  bmpRead();
  readGPS();
  readSGP();
  String timestamp = getTimestamp(); 
  startCapture();
  waitForCaptureComplete();
  //sendCapturedImageOverSerial();

  s8Read();
  readAnalogSensors();

  writeDataToSD();
  saveImageToSD();
  
  baterryPercentage();
  //tone(buzzerPin, 8000);  
  //delay(500);
  //noTone(buzzerPin);      
  lora_com();
  delay(1000);
}


  void initializeCamera() {
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
    pressure=(bmp.pressure/100);
    Serial.print(pressure);
    Serial.println(" hPa");

    Serial.print("Altitude from sea level = ");
    Serial.print(bmp.readAltitude(1013.25));
    pressureAltitude=bmp.readAltitude(1013.25);
    Serial.println(" m");

    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude(startingPressure));
    pressureAltSeaLevel=bmp.readAltitude(startingPressure);
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
      latitude=gps.location.lat();
      Serial.print(" Longitude: ");
      Serial.print(gps.location.lng(), 6);
      longitude=gps.location.lng();
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
        CO2=co2Concentration;
        
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
    if (CO < 0)
    {
      CO=0;
    }
    Serial.print("CO Concentration: ");
    Serial.print(concentration);
    Serial.print(" ppm ");
    Serial.println(adc0);

    adc0 = ads.readADC_SingleEnded(1);
    voltage = adc0 * 0.000125;  // convert ADC value to voltage
  
    concentration=voltage/0.02;

    SO2=concentration;
    if (SO2 < 0)
    {
      SO2=0;
    }
    Serial.print("SO2 Concentration: ");
    Serial.print(concentration);
    Serial.print(" ppm ");
    Serial.println(adc0);

    adc0 = ads.readADC_SingleEnded(2);
    voltage = adc0 * 0.000125;  // convert ADC value to voltage
  
    concentration=voltage/0.03;
    
    NO2=concentration;
    if (NO2 < 0)
    {
      NO2=0;
    }
    Serial.print("NO2 Concentration: ");
    Serial.print(concentration);
    Serial.print(" ppm ");
    Serial.println(adc0);

    adc0 = ads.readADC_SingleEnded(3);
    Serial.print("CH4 ADC value: ");
    CH4=adc0; //add calculation after calibration IMPORTANT
    Serial.println(adc0);
  }
void writeDataToSD() {
  String timestamp = getTimestamp(); 
  if (myFile.open("data.csv", O_WRITE | O_APPEND)) {
    // Write data to file (replace timestamp with actual timestamp generation logic)
    myFile.print(timestamp);  // Placeholder for timestamp
    myFile.print(", ");
    myFile.print(temperature);
    myFile.print(", ");
    Serial.println(pressure);
    myFile.print(", ");
    myFile.print(gpsAltitude);
    myFile.print(", ");
    myFile.print(pressureAltitude);
    myFile.print(", ");
    myFile.print(gpsAltSeaLevel);
    myFile.print(", ");
    myFile.print(pressureAltSeaLevel);
    myFile.print(", ");
    myFile.print(verticalSpeed);
    myFile.print(", ");
    myFile.print(speed);
    myFile.print(", ");
    myFile.print(battery);
    myFile.print(", ");
    myFile.print(latitude, 6);  // 6 decimal places for latitude
    myFile.print(", ");
    myFile.print(longitude, 6);  // 6 decimal places for longitude
    myFile.print(", ");
    myFile.print(predictedLongitude, 6);
    myFile.print(", ");
    myFile.print(predictedLatitude, 6);
    myFile.print(", ");
    myFile.print(CO2);
    myFile.print(", ");
    myFile.print(CO);
    myFile.print(", ");
    myFile.print(CH4);
    myFile.print(", ");
    myFile.print(NO2);
    myFile.print(", ");
    myFile.print(SO2);
    myFile.print(", ");
    myFile.print(TVOC);
    myFile.println(", ");

    
    // Sync to ensure data is written to SD card
    myFile.sync();
    
    // Close the file
    myFile.close();
    
    Serial.println("Data written to SD card.");
  } else {
    Serial.println("Failed to open file for appending.");
  }
  
}

void saveImageToSD() {
  String timestamp = getTimestamp(); 
  char filename[32];
  snprintf(filename, sizeof(filename), "%s.jpg", timestamp.c_str());

  if (!myFile.open(filename, O_WRITE | O_CREAT | O_TRUNC)) {
    Serial.println("File open failed");
    return;
  }

  myCAM.CS_LOW();
  SPI.transfer(BURST_FIFO_READ);

  myCAM.set_fifo_burst(); 

  const uint8_t BURST_SIZE = 128;
  uint16_t buf[BURST_SIZE];
  uint32_t len = myCAM.read_fifo_length();
  uint32_t remaining = len;

  while (remaining > 0) {
    uint8_t toRead = remaining >= BURST_SIZE ? BURST_SIZE : remaining;
    for (uint8_t i = 0; i < toRead; i++) {
      buf[i] = SPI.transfer(0x00);
    }
    myFile.write(buf, toRead);
    remaining -= toRead;
  }

  myFile.sync();   // Ensure buffer is flushed
  myFile.close();  // Very important!
  myCAM.CS_HIGH();

  Serial.print("Saved image as: ");
  Serial.println(filename);
}


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
  char timestamp[20]; 
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

void handleCommand(String cmd) {
    Serial.println("hanlde command triggered!");
    int commaIndex = cmd.indexOf(',');
    if (commaIndex == -1) return;
  
    String command = cmd.substring(commaIndex + 1);
  
    if (command == "WIFI") {
      Serial.println("Connecting to wifi and ftp server...");
      wifi_ftp_transfer();
    }
    else if(command == "WIFI_NO"){
      if(WiFi.status() == WL_CONNECTED){  // wifi connection is active
    // ditch the existing connection
    WiFi.disconnect();
    // allow some time for the connection to be fully dropped, important!
    delay(100);
    // check if the connection was really dropped
    if(WiFi.status() == WL_CONNECTED){
      Serial.println("Connection is still alive...");
    }else{
      Serial.println("Connection successfully terminated.");
      Serial.println(WiFi.localIP());
    }
  }else{  // there was no wifi connection to begin with, so nothing to disconnect
    Serial.println("No active connection was found to be terminated.");
  }
  
    }
  }
  
  void wifi_ftp_transfer(){
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print('.');
      delay(1000);
    }
  
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  
    Serial.print("RRSI: ");
    Serial.println(WiFi.RSSI());
  
    delay(500);
  
    ftp.OpenConnection();
  
    // change to dir where to store
    ftp.ChangeWorkDir("/data");  
  
    // create the file new and write a string into it
    ftp.InitFile("Type A");
    ftp.NewFile("hello_world.csv"); //<- .csv in this exmple - can be .txt ot other
    ftp.Write("Hello World");
    ftp.CloseFile();
  
    ftp.CloseConnection();
    Serial.println("All files uploaded successfully!");
  }

  void lora_com(){
    Serial.println("\n\n---Transmitting--");
  Serial.print(F("[SX1276] Transmitting packet ... "));
  
  TelemetryPacket packet;
  packet.packetID = count++;
  packet.temperature = temperature;
  packet.pressure = pressure;
  packet.gpsAltitude = gpsAltitude; 
  packet.pressureAltitude = pressureAltitude;
  packet.gpsAltSeaLevel = gpsAltSeaLevel; 
  packet.pressureAltSeaLevel = pressureAltSeaLevel;
  packet.verticalSpeed = verticalSpeed; 
  packet.horizontalSpeed = speed;
  packet.battery = battery;
  packet.latitude = latitude;
  packet.longitude = longitude;
  packet.predictedLongitude = 98;
  packet.predictedLatitude = 74;
  packet.CO2 = CO2;
  packet.CO = CO;
  packet.CH4 = CH4;
  packet.NO2 = NO2;
  packet.SO2 = SO2;
  packet.TVOC = TVOC;

  int state = radio.transmit((uint8_t*)&packet, sizeof(TelemetryPacket));

  uint8_t* ptr = (uint8_t*)&packet;
  size_t size = sizeof(packet);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("\n[SX1276] success!"));
    Serial.print("[SX1276] Sent:\t\t");
        for (size_t i = 0; i < size; ++i) {
            Serial.print("Byte ");
            Serial.print(i);
            Serial.print(": 0x");
            if (ptr[i] < 0x10) Serial.print("0");  // leading zero for single-digit hex
            Serial.println(ptr[i], HEX);
        }
    Serial.println();
    Serial.print(F("[SX1276] Datarate:\t\t"));
    Serial.print(radio.getDataRate());  
    Serial.println(F(" bps"));

    // Listen for ACK after sending
    Serial.println("[SX1276] Waiting for ACK...");
    radio.startReceive();
    unsigned long start = millis();
    while (millis() - start < 300) {  
      if (radio.available()) {
        String ack;
        int ackState = radio.readData(ack);
        if (ackState == RADIOLIB_ERR_NONE) {
          Serial.print("[SX1276] Received ACK: ");
          Serial.println(ack);
        } else {
          Serial.print("[SX1276] Failed to read ACK, code ");
          Serial.println(ackState);
        }
        break;
      }
    }
  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    Serial.println(F("too long!"));
  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    Serial.println(F("timeout!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  String incoming_data;

  if (receivedFlag) {
    receivedFlag = false;
    int state = radio.readData(incoming_data);

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("\n\n---Receiving---");
      Serial.print(F("[SX1276] Data:\t\t"));
      Serial.println(incoming_data);
      Serial.print(F("[SX1276] RSSI:\t\t"));
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));
      Serial.print(F("[SX1276] SNR:\t\t"));
      Serial.print(radio.getSNR());
      Serial.println(F(" dB"));
      Serial.print(F("[SX1276] Frequency error:\t"));
      Serial.print(radio.getFrequencyError());
      Serial.println(F(" Hz"));
    } else {
      Serial.print(F("[SX1276] Read failed, code "));
      Serial.println(state);
    }

  }

  handleCommand(incoming_data);
  }