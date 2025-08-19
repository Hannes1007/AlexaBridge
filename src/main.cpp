#include <Arduino.h>

#ifdef ARDUINO_ARCH_ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
#endif

#include <Espalexa.h>
#include <WiFiManager.h>

// Pin-Konstanten
const int RELAY_PIN = D6;
const int WIFI_RESET_PIN = D1;

// Globale Variablen
Espalexa espalexa;
bool lampState = false;

// Callback für Espalexa-Gerät
void lampControl(uint8_t brightness) 
{
  lampState = (brightness == 255);
  digitalWrite(RELAY_PIN, lampState ? HIGH : LOW);
}

// Espalexa-Geräte hinzufügen
void addDevices() 
{
  espalexa.addDevice("espAlexaTest", lampControl);
  espalexa.begin();
}

void setup() 
{
  Serial.begin(115200);

  pinMode(WIFI_RESET_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Relais initial aus

  WiFiManager wm;

  // Prüfen, ob Reset-Button beim Start gedrückt wird
  if (digitalRead(WIFI_RESET_PIN) == HIGH) {
    Serial.println("Button gedrückt -> WLAN-Daten löschen");
    wm.resetSettings();
    ESP.restart();
  }

  // AutoConnect startet AP, wenn keine gültigen Credentials vorhanden sind
  if (!wm.autoConnect("ESP_PW_12345678", "12345678")) {
    Serial.println("Keine Verbindung hergestellt, Neustart...");
    ESP.restart();
  }

  Serial.println("Verbunden mit: " + WiFi.SSID());

  addDevices();
}

void loop() 
{
  espalexa.loop(); // Espalexa am Leben halten
  delay(2);
}


