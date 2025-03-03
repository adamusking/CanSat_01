#include <Arduino.h>
#include <RadioLib.h>

#define retries 2
#define receive_window 150

SX1276 radio = new Module(10, 2, 9, 3);

uint16_t packetID = 0;
bool mode = true;

void setup(){
    Serial.begin(115200);

    Serial.println("LoRa initializing ... ");
    int modulation = radio.begin(868.0, 500.0, 7, 5, 0x3F, 17, 8, 0);

    if(modulation != RADIOLIB_ERR_NONE){
        Serial.println("Lora failed, code ");
        Serial.println(modulation);

        while (true) {
            delay(10);
        }
    }
    else{
        Serial.println("Lora initialized successfully!");
    }

    radio.setCRC(true);


}

void loop(){
    packetID++;
    String data = "ID" + String(packetID) + ",Hello World!";

    int state = radio.transmit(data);

    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("Packet sent, waiting for ACK");
      } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
        Serial.println("Packet too long!");
      } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
        Serial.println("Timed out while transmitting!");
      } else {
        Serial.println("Failed to sent packet, code ");
        Serial.println(state);
      }

      unsigned long start_time = millis();
      bool ackReceived = false;
      String ack;

      while(millis() - start_time < receive_window){
        int rxState = radio.receive(ack);

            if (rxState == RADIOLIB_ERR_NONE && ack == "ACK" + String(packetID)) {
                Serial.println("ACK received!");
                ackReceived = true;
                break;
            }
      }

      if(!ackReceived){
        Serial.println("No ACK received, retrying");
        packetID++;

        for(int i = 0; i < retries; i++){
            Serial.print("Retry ");
            Serial.println(i + 1);
            radio.transmit(data);
            delay(50);

            int RetryState = radio.receive(ack);

                if (RetryState == RADIOLIB_ERR_NONE && ack == "ACK" + String(packetID)) {
                    Serial.println("ACK received on retry!");
                    break;
                }
        }
      }
      else{
        Serial.println("Packet lost");
      }

      int commaIndex = ack.indexOf(',') + 1;
      String command = ack.substring(commaIndex);

      if (command == "LORA") {
        Serial.println("Switching to LoRa mode...");
        radio.begin(868.0, 500.0, 7, 5, 0x3F, 17, 8, 0);
        radio.setCRC(true);
        mode = true;
      } 
      else if (command == "FSK") {
        Serial.println("Switching to FSK mode...");
        radio.beginFSK(868.0, 300.0, 100.0, 17, 8, false);
        radio.setCRC(true);
        mode = false;
    }
    else if(command == "STAND"){
        radio.standby();
    }
}