#include <Arduino.h>
#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include <Espalexa.h>

const char* ssid = "";        // Replace with your Wi-Fi SSID
const char* password = ""; // Replace with your Wi-Fi password

Espalexa espalexa; // Create an instance of the Espalexa class
bool lampState = false; // Current state of the lamp
const int relayPin = D6;

void lampControl(uint8_t brightness) {
  lampState = brightness == 255;
  digitalWrite(relayPin, lampState ? HIGH : LOW); // Control the relay
}

void addDevices(){
  // Define your devices here.
  espalexa.addDevice("espAlexaTest", lampControl);
  espalexa.begin();
}

void setup() {
  Serial.begin(9600);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW); // Ensure the relay is off initially

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to Wi-Fi");

  addDevices();
}

void loop() {
  espalexa.loop(); // Keep the library running
  delay(2);
}