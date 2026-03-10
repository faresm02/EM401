#include <HardwareSerial.h>

// #define RX_PIN 19
// #define TX_PIN 17

#define RX_PIN 3
#define TX_PIN 4

HardwareSerial HC12(1);
  //17, 19); // HC-12 TX Pin, HC-12 RX Pin

void setup() {
  vTaskDelay(1000/portTICK_PERIOD_MS);
  Serial.begin(9600);             // Serial port to computer
  HC12.begin(9600,SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.print("HC12 TEST");

}

void loop() {
  while (HC12.available()) {        // If HC-12 has data
    Serial.write(HC12.read());      // Send the data to Serial monitor
  }
  while (Serial.available()) {      // If Serial monitor has data
    HC12.write(Serial.read());      // Send that data to HC-12
  }
}