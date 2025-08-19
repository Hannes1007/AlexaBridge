#include <Arduino.h>

#ifdef ARDUINO_ARCH_ESP32
  #include <WiFi.h>
  #include <driver/i2s.h>
#else
  #include <ESP8266WiFi.h>
#endif

#include <Espalexa.h>
#include <WiFiManager.h>

// Pin-Konstanten
const int RELAY_PIN = 32; // Beispiel: GPIO32 für ESP32
const int WIFI_RESET_PIN = 17; // Beispiel: GPIO25 für ESP32

// I2S Mikrofon-Pins (INMP441)
#define I2S_WS 19
#define I2S_SD 5
#define I2S_SCK 18
#define I2S_PORT I2S_NUM_0
#define bufferLen 64
int16_t sBuffer[bufferLen];

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

// I2S Initialisierung
void i2s_install() {
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = bufferLen,
    .use_apll = false
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin() {
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };
  i2s_set_pin(I2S_PORT, &pin_config);
}

void setup() 
{
  Serial.begin(115200);
  Serial.println(" ");

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

#ifdef ARDUINO_ARCH_ESP32
  delay(1000);
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
  delay(500);
#endif
}

void loop() 
{
#ifdef ARDUINO_ARCH_ESP32
  // Mikrofon-Daten für Serial Plotter
  int rangelimit = 3000;
  Serial.print(rangelimit * -1);
  Serial.print(" ");
  Serial.print(rangelimit);
  Serial.print(" ");

  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);

  if (result == ESP_OK)
  {
    int16_t samples_read = bytesIn / 8;
    if (samples_read > 0) {
      float mean = 0;
      for (int16_t i = 0; i < samples_read; ++i) {
        mean += (sBuffer[i]);
      }
      mean /= samples_read;
      Serial.println(mean);
    }
  }
#endif

  espalexa.loop(); // Espalexa am Leben halten
}


