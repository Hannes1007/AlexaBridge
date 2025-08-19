#include <Arduino.h>
#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include <Espalexa.h>
#include <WiFiManager.h>

Espalexa espalexa; // Create an instance of the Espalexa class
bool lampState = false; // Current state of the lamp
const int relayPin = D6;
const int wifiResetPin = D1;

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
  Serial.begin(115200);

  pinMode(wifiResetPin, INPUT);

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW); // Ensure the relay is off initially



   WiFiManager wm;

  // prüfen ob der Button beim Start gedrückt wird
  int val = digitalRead(wifiResetPin);   // read the input pin

  if (val == 1) {
    Serial.println("Button gedrückt -> WLAN-Daten löschen");
    wm.resetSettings();   // löscht gespeicherte SSID + Passwort
    ESP.restart();        // Neustart
  }

  // AutoConnect startet AP, wenn keine gültigen Credentials vorhanden sind
  if (!wm.autoConnect("ESP_PW_12345678", "12345678")) {
    Serial.println("Keine Verbindung hergestellt, Neustart...");
    ESP.restart();
  }
  Serial.println("Verbunden mit: " + WiFi.SSID());



  addDevices();
  //espalexa.getDevice(1)->setState(true);
}

void loop() {
  espalexa.loop(); // Keep the library running
  delay(2);
}


