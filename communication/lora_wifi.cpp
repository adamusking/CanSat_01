#include <RadioLib.h>
#include <SPI.h>
#include <WiFi.h>
#include <ESP32_FTPClient.h>

#define ss 47
#define rst 41
#define dio0 19
#define mosi 36
#define miso 42
#define sck 40

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

void handleCommand(String cmd);
void wifi_ftp_transfer();

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
  Serial.begin(9600);
  while (!Serial);
  Serial.println("LoRa Transceiver");

  SPI.begin(sck, miso, mosi, ss);

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

void loop() {
  Serial.println("\n\n---Transmitting--");
  Serial.print(F("[SX1276] Transmitting packet ... "));
  
  TelemetryPacket packet;
  packet.packetID = count++;
  packet.temperature = 2500;
  packet.pressure = 9;
  packet.gpsAltitude = 6; 
  packet.pressureAltitude = 54;
  packet.gpsAltSeaLevel = 54; 
  packet.pressureAltSeaLevel = 84;
  packet.verticalSpeed = 88; 
  packet.horizontalSpeed = 12;
  packet.battery = 36;
  packet.latitude = 98;
  packet.longitude = 30;
  packet.predictedLongitude = 98;
  packet.predictedLatitude = 74;
  packet.CO2 = 3;
  packet.CO = 6;
  packet.CH4 = 8;
  packet.NO2 = 7;
  packet.SO2 = 74;
  packet.TVOC = 2;

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

  delay(1000);
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