#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

#define ss 47
#define rst 41
#define dio0 38
#define mosi 36
#define miso 37
#define sck 40 
#define retries 2
#define receive_window 150
#define stand_by_time 5000

SX1276 radio = SX1276(new Module(ss, dio0, rst));

uint16_t packetID = 0;
bool mode = true;
bool standby = false;
int transmissionState;

volatile bool transmittedFlag = false;
volatile bool receivedFlag = false;

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setTransmitFlag(void) {
    transmittedFlag = true;
}

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setReceiveFlag(void) {
    receivedFlag = true;
}

void handleCommand(String cmd) {
    int commaIndex = cmd.indexOf(',');
    if (commaIndex == -1) return;

    String command = cmd.substring(commaIndex + 1);

    if (command == "LORA") {
        Serial.println("Switching to LoRa mode...");
        radio.begin(866.0, 125.0, 7, 5, 0xA5, 17, 12, 0);
        radio.setCRC(true);
        mode = true;
    } else if (command == "FSK") {
        Serial.println("Switching to FSK mode...");
        radio.beginFSK(868.0, 300.0, 100.0, 17, 8, false);
        radio.setCRC(true);
        mode = false;
    } else if (command == "STAND") {
        Serial.println("Entering standby mode...");
        radio.standby();
        standby = true;
    } else if (command == "WAKE") {
        Serial.println("Exiting standby mode...");
        standby = false;
    }
}

void setup() {
    Serial.begin(9600);
    while (!Serial);
    Serial.println("LoRa initializing ...");
    SPI.begin(sck, miso, mosi, ss);

    int state = radio.begin(866.0, 125.0, 7, 5, 0xA5, 17, 12, 0);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("LoRa initialization failed, code ");
        Serial.println(state);
        while (true) delay(1000);
    }
    Serial.println("LoRa initialized successfully!");
    radio.setPacketSentAction(setTransmitFlag);
    radio.setPacketReceivedAction(setReceiveFlag);

    radio.setCRC(true);
}

void loop() {
    if (standby) {
        radio.startReceive();
        if (receivedFlag) {
            receivedFlag = false;
            String ack;
            int state = radio.readData(ack);
            if (state == RADIOLIB_ERR_NONE) {
                handleCommand(ack);
            }
        }
        return;
    }

    packetID++;
    String data = "ID:" + String(packetID) + ",Hello World!";

    Serial.println("Sending packet...");
    transmissionState = radio.startTransmit(data);

    if (transmissionState == RADIOLIB_ERR_NONE) {
        Serial.println("Packet sent, waiting for ACK");
    } else {
        Serial.print("Transmission failed, code ");
        Serial.println(transmissionState);
    }

    unsigned long start_time = millis();
    bool ackReceived = false;
    String ack;

    while (millis() - start_time < receive_window) {
        if (receivedFlag) {
            receivedFlag = false;
            int state = radio.readData(ack);
            if (state == RADIOLIB_ERR_NONE) {
                int commaIndex = ack.indexOf(',');
                if (commaIndex > 0 && ack.substring(0, commaIndex) == "ACK" + String(packetID)) {
                    Serial.println("ACK received!");
                    ackReceived = true;
                    break;
                }
            }
        }
    }

    if (!ackReceived) {
        Serial.println("No ACK received, retrying...");
        for (int i = 0; i < retries; i++) {
            Serial.print("Retry ");
            Serial.println(i + 1);
            radio.startTransmit(data);
            delay(50 * pow(2, i));
            if (receivedFlag) {
                receivedFlag = false;
                int state = radio.readData(ack);
                if (state == RADIOLIB_ERR_NONE) {
                    int commaIndex = ack.indexOf(',');
                    if (commaIndex > 0 && ack.substring(0, commaIndex) == "ACK" + String(packetID)) {
                        Serial.println("ACK received on retry!");
                        ackReceived = true;
                        break;
                    }
                }
            }
        }
    }

    if (!ackReceived) {
        Serial.println("Packet lost");
        radio.startReceive(); // Ensure we enter listening mode
        return;
    }

    if (ack.length() > 0) {
        handleCommand(ack);
    }
}
