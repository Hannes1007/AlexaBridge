#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#ifdef ARDUINO_ARCH_ESP32
  #include <WiFi.h>
  #include <driver/i2s.h>
#else
  #include <ESP8266WiFi.h>
#endif

#include <WiFiManager.h>
#include <fauxmoESP.h>
fauxmoESP fauxmo;

// Alexa-Geräte
#define ID_fog            "bulliNebelmaschine"
#define ID_schlussLinks   "bulliSchlussLinks"
#define ID_partyMode      "bulliPartyModus"

int pos = 0;    // variable to store the servo position

// Pin-Konstanten
const int WIFI_RESET_PIN = 4; // Beispiel: GPIO25 für ESP32

// I2S Mikrofon-Pins (INMP441)
#define I2S_WS 23
#define I2S_SD 21
#define I2S_SCK 22
#define I2S_PORT I2S_NUM_0
#define bufferLen 64
int16_t sBuffer[bufferLen];

// Globale Variablen
bool partyModeActive = false;   // Alexa schaltet diesen Modus

// PCA9685-Objekt
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// LED-Kanäle auf dem PCA9685 (0-15)
const uint8_t RL_SCHLUSS_LINKS      = 0; // Kanal 0
const uint8_t RL_BREMS_LINKS        = 1; // Kanal 1
const uint8_t RL_BLINKER_LINKS      = 2; // Kanal 2
const uint8_t RL_RUECKFAHR_LINKS    = 3; // Kanal 3
const uint8_t RL_NEBEL_LINKS        = 4; // Kanal 4

const uint8_t RL_SCHLUSS_RECHTS     = 8; // Kanal 8
const uint8_t RL_BREMS_RECHTS       = 9; // Kanal 9
const uint8_t RL_BLINKER_RECHTS     = 10; // Kanal 10
const uint8_t RL_RUECKFAHR_RECHTS   = 11; // Kanal 11
// Keine Nebelschlussleuchte rechts

// Dynamische Max-Werte für AGC
float dynamicMax = 1000.0;
const float decay = 0.995;
const float attack = 1.2;

// Alle 9 Kanäle in ein Array packen
const uint8_t ledChannels[9] = {
  RL_SCHLUSS_LINKS, RL_BREMS_LINKS, RL_BLINKER_LINKS, RL_RUECKFAHR_LINKS, RL_NEBEL_LINKS,
  RL_SCHLUSS_RECHTS, RL_BREMS_RECHTS, RL_BLINKER_RECHTS, RL_RUECKFAHR_RECHTS
};

void lightOrgan(float sample) {
  // 1) Betrag nehmen
  float level = abs(sample);

  // 2) Dynamischen Maximalwert anpassen
  if (level > dynamicMax) {
    dynamicMax = level * attack;
  } else {
    dynamicMax *= decay;
  }
  if (dynamicMax < 500) dynamicMax = 500;

  // 3) Zahl der aktiven LEDs bestimmen
  int ledsOn = map(level, 0, (int)dynamicMax, 0, 9);
  if (ledsOn > 9) ledsOn = 9;

  // 4) Alle LEDs durchgehen
  for (int i = 0; i < 9; i++) {
    if (i < ledsOn) {
      uint16_t brightness = random(2000, 4095);
      pwm.setPWM(ledChannels[random(0, 9)], 0, brightness);
    } else {
      pwm.setPWM(ledChannels[i], 0, 0);
    }
  }
}

int angleToPulse(int ang) {
  return map(ang, 0, 180, 150, 600);
}

// Schlusslicht Links steuern
void rlSchlussLinksControl(uint8_t brightness) {
  if (brightness == 0) {
    pwm.setPWM(RL_SCHLUSS_LINKS, 0, 0);  // AUS
  } else {
    uint16_t pwmValue = map(brightness, 0, 255, 0, 4095);
    pwm.setPWM(RL_SCHLUSS_LINKS, 0, pwmValue);
  }
}

// Nebelmaschine
int fogChannel = 15;
bool fogActive = false;
unsigned long fogStartTime = 0;
unsigned long fogDuration = 0;
int onPosition = angleToPulse(90);
int offPosition = angleToPulse(60);

void fogMachineControl(uint8_t duration) {
  duration = duration / 30;

  if (duration > 0) {
    fogActive = true;
    fogStartTime = millis();
    fogDuration = duration * 1000UL;
    Serial.println("Nebelmaschine aktiviert für " + String(duration) + " Sekunden");

    pwm.setPWM(fogChannel, 0, onPosition);
  } else {
    fogActive = false;
    Serial.println("Nebelmaschine deaktiviert");
    pwm.setPWM(fogChannel, 0, offPosition);
  }
}

void handleFogMachine() {
  if (fogActive && millis() - fogStartTime >= fogDuration) {
    fogActive = false;
    pwm.setPWM(fogChannel, 0, offPosition);
    Serial.println("Nebelmaschine automatisch ausgeschaltet");
  }
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

void setup() {
  Serial.begin(115200);
  Serial.println(" ");

  pinMode(WIFI_RESET_PIN, INPUT);

  Wire.begin(32, 33);

  pwm.begin();
  pwm.setPWMFreq(50);

  WiFiManager wm;

  if (digitalRead(WIFI_RESET_PIN) == HIGH) {
    Serial.println("Button gedrückt -> WLAN-Daten löschen");
    wm.resetSettings();
    ESP.restart();
  }

  if (!wm.autoConnect("ESP_PW_12345678", "12345678")) {
    Serial.println("Keine Verbindung hergestellt, Neustart...");
    ESP.restart();
  }

  Serial.println("Verbunden mit: " + WiFi.SSID());

#ifdef ARDUINO_ARCH_ESP32
  delay(1000);
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
  delay(500);
#endif

  // Alexa-Geräte registrieren
  fauxmo.addDevice(ID_fog);
  fauxmo.addDevice(ID_schlussLinks);
  fauxmo.addDevice(ID_partyMode);

  fauxmo.setPort(80);
  fauxmo.enable(true);

  fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state, unsigned char value) {
    Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n",
                  device_id, device_name, state ? "ON" : "OFF", value);

    if (strcmp(device_name, ID_fog) == 0) {
      fogMachineControl(value);
    } else if (strcmp(device_name, ID_schlussLinks) == 0) {
      rlSchlussLinksControl(state ? value : 0);
    } else if (strcmp(device_name, ID_partyMode) == 0) {
      partyModeActive = state;
      Serial.println(partyModeActive ? "🎉 Party-Modus EIN" : "⏹ Party-Modus AUS");
    }
  });
}

void loop() {
#ifdef ARDUINO_ARCH_ESP32
  int rangelimit = 3000;
  Serial.print(rangelimit * -1);
  Serial.print(" ");
  Serial.print(rangelimit);
  Serial.print(" ");

  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);

  if (result == ESP_OK) {
    int16_t samples_read = bytesIn / sizeof(int16_t);
    if (samples_read > 0) {
      float mean = 0;
      for (int16_t i = 0; i < samples_read; ++i) {
        mean += (sBuffer[i]);
      }
      mean /= samples_read;
      Serial.println(mean);

      if (partyModeActive) {
        lightOrgan(mean);   // 🎶 Nur im Party-Modus aktiv
      }
    }
  }
#endif

  fauxmo.handle();
  handleFogMachine();
}
