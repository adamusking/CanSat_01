#include "s8_uart.h"

// Define the UART pins
#define S8_RX_PIN 16
#define S8_TX_PIN 17

HardwareSerial S8_serial(1);

S8_UART sensor_S8(S8_serial);

void setup() {
  Serial.begin(115200); // start the serial communication for debugging

  S8_serial.begin(S8_BAUDRATE, SERIAL_8N1, S8_RX_PIN, S8_TX_PIN);   // start the communication with the sensor

  // Check if the sensor is available
  char firmware_version[16];
  sensor_S8.get_firmware_version(firmware_version);
  if (strlen(firmware_version) == 0) {
    Serial.println("CO2 sensor not found!");
    while (1) {
      delay(1000);
    }
  }

  // display sensor information
  Serial.print("Firmware version: ");
  Serial.println(firmware_version);
  uint16_t sensor_id = sensor_S8.get_sensor_ID();
  Serial.print("Sensor ID: 0x");
  Serial.println(sensor_id, HEX);
}

void loop() {
  // read CO2 concentration
  int co2_concentration = sensor_S8.get_co2();
  if (co2_concentration >= 0) {
    Serial.print("CO2 Concentration: ");
    Serial.print(co2_concentration);
    Serial.println(" ppm");
  } else {
    Serial.println("Failed to read CO2 concentration.");
  }

  delay(2000);
}
