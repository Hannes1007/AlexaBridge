#include <Arduino.h>

#ifdef ARDUINO_ARCH_ESP32
  #include <WiFi.h>
  #include <driver/i2s.h>

  // I2S-Konstanten
  #define SAMPLE_BUFFER_SIZE 512
  #define SAMPLE_RATE 8000
  #define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
  #define I2S_MIC_SERIAL_CLOCK GPIO_NUM_26
  #define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_25
  #define I2S_MIC_SERIAL_DATA GPIO_NUM_22

  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 4,
      .dma_buf_len = 1024,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0};

  i2s_pin_config_t i2s_mic_pins = {
      .bck_io_num = I2S_MIC_SERIAL_CLOCK,
      .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = I2S_MIC_SERIAL_DATA};

  int32_t raw_samples[SAMPLE_BUFFER_SIZE];
#else
  #include <ESP8266WiFi.h>
#endif

#include <Espalexa.h>
#include <WiFiManager.h>

// Pin-Konstanten
const int RELAY_PIN = 32; // Beispiel: GPIO32 für ESP32
const int WIFI_RESET_PIN = 17; // Beispiel: GPIO25 für ESP32

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

#ifdef ARDUINO_ARCH_ESP32
  // I2S initialisieren
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &i2s_mic_pins);
#endif

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
#ifdef ARDUINO_ARCH_ESP32
  // I2S-Mikrofon auslesen und Werte ausgeben
  size_t bytes_read = 0;
  i2s_read(I2S_NUM_0, raw_samples, sizeof(int32_t) * SAMPLE_BUFFER_SIZE, &bytes_read, portMAX_DELAY);
  int samples_read = bytes_read / sizeof(int32_t);
  for (int i = 0; i < samples_read; i++) {
    Serial.printf("%ld\n", raw_samples[i]);
  }
#endif

  espalexa.loop(); // Espalexa am Leben halten
  delay(2);
}


