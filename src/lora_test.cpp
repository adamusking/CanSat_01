
#include <RadioLib.h>

// LoRa module setup (ESP32)
SX1276 radio = SX1276(new Module(10, 9, 3));

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing LoRa...");

    // Configure LoRa parameters to match the Python receiver
    int status = radio.begin(868.0, 500.0, 7, 5, 0x3F, 17, 8, 0);

    if (status != RADIOLIB_ERR_NONE) {
        Serial.print("LoRa initialization failed, code: ");
        Serial.println(status);
        while (true) delay(1000);
    }

    Serial.println("LoRa initialized successfully!");
    radio.setCRC(true);
}

void loop() {
    String message = "Hello from ESP32!";
    int state = radio.transmit(message);

    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("Message sent!");
    } else {
        Serial.print("Transmission failed, code: ");
        Serial.println(state);
    }

    delay(1000); // Send message every second
}
