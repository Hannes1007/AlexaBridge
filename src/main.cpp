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

// =======================================================
// === Alexa-Geräte ===
#define ID_FOG            "bulliNebelmaschine"

#define ID_SCHLUSS_LINKS  "bulliSchlussLinks"
#define ID_BREMS_LINKS    "bulliBremsLinks"
#define ID_BLINKER_LINKS  "bulliBlinkerLinks"
#define ID_RUECK_LINKS    "bulliRueckLinks"
#define ID_NEBEL_LINKS    "bulliNebelLinks"

#define ID_SCHLUSS_RECHTS "bulliSchlussRechts"
#define ID_BREMS_RECHTS   "bulliBremsRechts"
#define ID_BLINKER_RECHTS "bulliBlinkerRechts"
#define ID_RUECK_RECHTS   "bulliRueckRechts"

#define ID_PARTY_MODE     "bulliPartyModus"

// =======================================================
// === Pin-Konstanten ===
const int WIFI_RESET_PIN = 4;  // WLAN Reset Button (ESP32 Beispiel GPIO4)

// I2S Mikrofon (INMP441)
#define I2S_WS    23
#define I2S_SD    21
#define I2S_SCK   22
#define I2S_PORT  I2S_NUM_0
#define BUFFER_LEN 64
int16_t sBuffer[BUFFER_LEN];

// =======================================================
// === PCA9685 LED / Servo ===
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// LED-Kanäle links
const uint8_t RL_SCHLUSS_LINKS      = 0;
const uint8_t RL_BREMS_LINKS        = 1;
const uint8_t RL_BLINKER_LINKS      = 2;
const uint8_t RL_RUECKFAHR_LINKS    = 3;
const uint8_t RL_NEBEL_LINKS        = 4;

// LED-Kanäle rechts
const uint8_t RL_SCHLUSS_RECHTS     = 8;
const uint8_t RL_BREMS_RECHTS       = 9;
const uint8_t RL_BLINKER_RECHTS     = 10;
const uint8_t RL_RUECKFAHR_RECHTS   = 11;

// =======================================================
// === Globale Variablen ===
bool partyModeActive = true; // Party-Modus standardmäßig an

float dynamicMax = 1000.0;
const float decay = 0.995;
const float attack = 1.2;

// Empfindlichkeit der Nebelmaschine im Party-Modus
float fogSensitivity = 1.0;

// =======================================================
// === Hilfsfunktionen: PWM Steuerung ===
void setLedPWM(uint8_t channel, uint8_t brightness) {
  if (brightness == 0) pwm.setPWM(channel, 0, 0);
  else pwm.setPWM(channel, 0, map(brightness, 0, 255, 0, 4095));
}

// Links
void rlSchlussLinksControl(uint8_t brightness)   { setLedPWM(RL_SCHLUSS_LINKS, brightness); }
void rlBremsLinksControl(uint8_t brightness)     { setLedPWM(RL_BREMS_LINKS, brightness); }
void rlBlinkerLinksControl(uint8_t brightness)   { setLedPWM(RL_BLINKER_LINKS, brightness); }
void rlRueckfahrLinksControl(uint8_t brightness) { setLedPWM(RL_RUECKFAHR_LINKS, brightness); }
void rlNebelLinksControl(uint8_t brightness)     { setLedPWM(RL_NEBEL_LINKS, brightness); }

// Rechts
void rlSchlussRechtsControl(uint8_t brightness)   { setLedPWM(RL_SCHLUSS_RECHTS, brightness); }
void rlBremsRechtsControl(uint8_t brightness)     { setLedPWM(RL_BREMS_RECHTS, brightness); }
void rlBlinkerRechtsControl(uint8_t brightness)   { setLedPWM(RL_BLINKER_RECHTS, brightness); }
void rlRueckfahrRechtsControl(uint8_t brightness) { setLedPWM(RL_RUECKFAHR_RECHTS, brightness); }

// =======================================================
// === Party-Modus Lichtorgel ===
void lightOrgan(float sample) {
  float level = abs(sample);

  if (level > dynamicMax) dynamicMax = level * attack;
  else dynamicMax *= decay;
  if (dynamicMax < 500) dynamicMax = 500;

  int ledsOn = map(level, 0, (int)dynamicMax, 0, 9);
  if (ledsOn > 9) ledsOn = 9;

  const uint8_t ledChannels[9] = {
    RL_SCHLUSS_LINKS, RL_BREMS_LINKS, RL_BLINKER_LINKS, RL_RUECKFAHR_LINKS, RL_NEBEL_LINKS,
    RL_SCHLUSS_RECHTS, RL_BREMS_RECHTS, RL_BLINKER_RECHTS, RL_RUECKFAHR_RECHTS
  };

  for (int i = 0; i < 9; i++) {
    if (i < ledsOn) pwm.setPWM(ledChannels[random(0,9)], 0, random(2000, 4095));
    else pwm.setPWM(ledChannels[i], 0, 0);
  }
}

// =======================================================
// === Nebelmaschine ===
struct FogMachine {
  int channel;
  int offPos;
  int onPos;
  int currentPos;
  bool active;
  unsigned long startTime;
  unsigned long duration;
  unsigned long lastTrigger;
  unsigned long minInterval;
};

FogMachine fog = {15, 150, 600, 150, false, 0, 0, 0, 2000}; // minInterval = 2 Sekunden

void triggerFog(unsigned long dur_ms) {
  if (millis() - fog.lastTrigger < fog.minInterval) return;
  fog.active = true;
  fog.startTime = millis();
  fog.duration = dur_ms;
  fog.lastTrigger = millis();
  Serial.println("💨 Nebelmaschine aktiviert für " + String(dur_ms) + " ms");
}

void updateFogMachine() {
  fog.offPos = map(90, 0, 180, 150, 600);
  fog.onPos  = map(60, 0, 180, 150, 600);

  int targetPos = fog.active ? fog.onPos : fog.offPos;
  int step = (targetPos - fog.currentPos) / 8;

  if (step != 0) {
    fog.currentPos += step;
    pwm.setPWM(fog.channel, 0, fog.currentPos);
    delay(20); // kurz warten, damit Servo Position erreicht
    pwm.setPWM(fog.channel, 0, 0); // Servo stromlos
  }

  if (fog.active && millis() - fog.startTime >= fog.duration) {
    fog.active = false;
    Serial.println("💨 Nebelmaschine automatisch ausgeschaltet");
  }
}

// =======================================================
// === I2S Mikrofon (ESP32) ===
#ifdef ARDUINO_ARCH_ESP32
void i2sInstall() {
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_LEN,
    .use_apll = false
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2sSetPin() {
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num  = I2S_WS,
    .data_out_num = -1,
    .data_in_num  = I2S_SD
  };
  i2s_set_pin(I2S_PORT, &pin_config);
}
#endif

// =======================================================
// === Setup ===
void setup() {
  Serial.begin(115200);
  pinMode(WIFI_RESET_PIN, INPUT);
  Wire.begin(32, 33);

  pwm.begin();
  pwm.setPWMFreq(50);

  // Servo initial in Aus-Position
  pwm.setPWM(fog.channel, 0, fog.offPos);

  // === WLAN Setup ===
  WiFiManager wm;
  if (digitalRead(WIFI_RESET_PIN) == HIGH) {
    wm.resetSettings();
    ESP.restart();
  }
  if (!wm.autoConnect("ESP_PW_12345678", "12345678")) ESP.restart();
  Serial.println("Verbunden mit: " + WiFi.SSID());

  // === I2S Setup ===
#ifdef ARDUINO_ARCH_ESP32
  delay(1000);
  i2sInstall();
  i2sSetPin();
#endif

  // === Alexa Setup ===
  fauxmo.addDevice(ID_FOG);

  fauxmo.addDevice(ID_SCHLUSS_LINKS);
  fauxmo.addDevice(ID_BREMS_LINKS);
  fauxmo.addDevice(ID_BLINKER_LINKS);
  fauxmo.addDevice(ID_RUECK_LINKS);
  fauxmo.addDevice(ID_NEBEL_LINKS);

  fauxmo.addDevice(ID_SCHLUSS_RECHTS);
  fauxmo.addDevice(ID_BREMS_RECHTS);
  fauxmo.addDevice(ID_BLINKER_RECHTS);
  fauxmo.addDevice(ID_RUECK_RECHTS);

  fauxmo.addDevice(ID_PARTY_MODE);

  fauxmo.setPort(80);
  fauxmo.enable(true);

  // Party-Modus direkt aktivieren
  partyModeActive = true;
  fauxmo.setState(ID_PARTY_MODE, true, 255);
  Serial.println("🎉 Party-Modus standardmäßig EIN beim Start");

  fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state, unsigned char value) {
    Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n",
                  device_id, device_name, state ? "ON" : "OFF", value);

    if (strcmp(device_name, ID_FOG) == 0) triggerFog(value * 10); // Einfache Steuerung
    else if (strcmp(device_name, ID_PARTY_MODE) == 0) {
      partyModeActive = state;
      Serial.println(partyModeActive ? "🎉 Party-Modus EIN" : "⏹ Party-Modus AUS");
    }

    else if (strcmp(device_name, ID_SCHLUSS_LINKS) == 0) rlSchlussLinksControl(state ? value : 0);
    else if (strcmp(device_name, ID_BREMS_LINKS) == 0)   rlBremsLinksControl(state ? value : 0);
    else if (strcmp(device_name, ID_BLINKER_LINKS) == 0) rlBlinkerLinksControl(state ? value : 0);
    else if (strcmp(device_name, ID_RUECK_LINKS) == 0)   rlRueckfahrLinksControl(state ? value : 0);
    else if (strcmp(device_name, ID_NEBEL_LINKS) == 0)   rlNebelLinksControl(state ? value : 0);

    else if (strcmp(device_name, ID_SCHLUSS_RECHTS) == 0) rlSchlussRechtsControl(state ? value : 0);
    else if (strcmp(device_name, ID_BREMS_RECHTS) == 0)   rlBremsRechtsControl(state ? value : 0);
    else if (strcmp(device_name, ID_BLINKER_RECHTS) == 0) rlBlinkerRechtsControl(state ? value : 0);
    else if (strcmp(device_name, ID_RUECK_RECHTS) == 0)   rlRueckfahrRechtsControl(state ? value : 0);
  });
}

// =======================================================
// === Loop ===
void loop() {
#ifdef ARDUINO_ARCH_ESP32
  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, BUFFER_LEN, &bytesIn, portMAX_DELAY);

  if (result == ESP_OK) {
    int samples_read = bytesIn / sizeof(int16_t);
    if (samples_read > 0) {
      // RMS berechnen
      float sum = 0;
      for (int i = 0; i < samples_read; i++) sum += sBuffer[i] * sBuffer[i];
      float rms = sqrt(sum / samples_read);

      if (partyModeActive) {
        lightOrgan(rms); // Lichtorgel

        float threshold = dynamicMax / fogSensitivity;
        if (rms > threshold) triggerFog(800); // Dauer 0,8s
      }
    }
  }
#endif

  updateFogMachine(); // immer aufrufen
  fauxmo.handle();
}
