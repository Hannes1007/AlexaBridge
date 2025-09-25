#include <Arduino.h>
#include <Wire.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>

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

#define ID_SCHLUSS_RECHTS "bulliSchlussRechts"
#define ID_BREMS_RECHTS   "bulliBremsRechts"
#define ID_BLINKER_RECHTS "bulliBlinkerRechts"
#define ID_RUECK_RECHTS   "bulliRueckRechts"
#define ID_NEBEL_RECHTS    "bulliNebelRechts"

#define ID_PARTY_MODE     "bulliPartyModus"

// =======================================================
// === Pin-Konstanten ===
const int WIFI_RESET_PIN = 4;  // WLAN Reset Button (ESP32 Beispiel GPIO4)
const int FOG_PIN = 32;         // Nebelmaschinen Servo (ESP32 Beispiel GPIO13)

// I2S Mikrofon (INMP441)
#define I2S_WS    23
#define I2S_SD    21
#define I2S_SCK   22
#define I2S_PORT  I2S_NUM_0
#define BUFFER_LEN 64
int16_t sBuffer[BUFFER_LEN];

// LED-Kanäle links (ESP32 Pins)
const uint8_t RL_SCHLUSS_LINKS      = 33;
const uint8_t RL_BREMS_LINKS        = 25;
const uint8_t RL_BLINKER_LINKS      = 26;
const uint8_t RL_RUECKFAHR_LINKS    = 27;

// LED-Kanäle rechts (ESP32 Pins)
const uint8_t RL_SCHLUSS_RECHTS     = 19;
const uint8_t RL_BREMS_RECHTS       = 18;
const uint8_t RL_BLINKER_RECHTS     = 5;
const uint8_t RL_RUECKFAHR_RECHTS   = 17;
const uint8_t RL_NEBEL_RECHTS       = 16;


// =======================================================
// === Globale Variablen ===
bool partyModeActive = false; // Party-Modus standardmäßig an

float dynamicMax = 1000.0;
const float decay = 0.995;
const float attack = 1.2;

// Empfindlichkeit der Nebelmaschine im Party-Modus
float fogSensitivity = 1.0;

// Für Party-Lichtorgel: Shuffle-Intervall global
static unsigned long lastShuffle = 0;
static unsigned long nextShuffleInterval = 2000;

// =======================================================
// === Hilfsfunktionen: PWM Steuerung ===
void setLedPWM(uint8_t pin, uint8_t brightness, uint8_t channel) {
  static bool pwmInit[16] = {false};
  if (!pwmInit[channel]) {
    ledcSetup(channel, 1000, 8); // Kanal zuerst initialisieren!
    ledcAttachPin(pin, channel);
    pwmInit[channel] = true;
  }
  ledcWrite(channel, brightness);
}

// Links
void rlSchlussLinksControl(uint8_t brightness)   { setLedPWM(RL_SCHLUSS_LINKS, brightness, 0); }
void rlBremsLinksControl(uint8_t brightness)     { setLedPWM(RL_BREMS_LINKS, brightness, 1); }
void rlBlinkerLinksControl(uint8_t brightness)   { setLedPWM(RL_BLINKER_LINKS, brightness, 2); }
void rlRueckfahrLinksControl(uint8_t brightness) { setLedPWM(RL_RUECKFAHR_LINKS, brightness, 3); }

// Rechts
void rlSchlussRechtsControl(uint8_t brightness)   { setLedPWM(RL_SCHLUSS_RECHTS, brightness, 4); }
void rlBremsRechtsControl(uint8_t brightness)     { setLedPWM(RL_BREMS_RECHTS, brightness, 5); }
void rlBlinkerRechtsControl(uint8_t brightness)   { setLedPWM(RL_BLINKER_RECHTS, brightness, 6); }
void rlRueckfahrRechtsControl(uint8_t brightness) { setLedPWM(RL_RUECKFAHR_RECHTS, brightness, 7); }
void rlNebelRechtsControl(uint8_t brightness)     { setLedPWM(RL_NEBEL_RECHTS, brightness, 8); }


// =======================================================
// === Party-Modus Lichtorgel ===
void lightOrgan(float sample) {
    static uint8_t ledOrder[9] = {0,1,2,3,4,5,6,7,8};

    // LED-Kanäle (wie gehabt)
    const uint8_t ledPins[9] = {
        RL_SCHLUSS_LINKS, RL_BREMS_LINKS, RL_BLINKER_LINKS, RL_RUECKFAHR_LINKS,
        RL_SCHLUSS_RECHTS, RL_BREMS_RECHTS, RL_BLINKER_RECHTS, RL_RUECKFAHR_RECHTS, RL_NEBEL_RECHTS
    };

    // Nur alle 2 bis 10 Sekunden neu mischen
    if (millis() - lastShuffle > nextShuffleInterval) {
        for (int i = 8; i > 0; i--) {
            int j = random(0, i + 1);
            uint8_t temp = ledOrder[i];
            ledOrder[i] = ledOrder[j];
            ledOrder[j] = temp;
        }
        lastShuffle = millis();
        nextShuffleInterval = random(2000, 10000); // 2 bis 10 Sekunden
    }

    float level = abs(sample);

    // Dynamik wie gehabt
    if (level > dynamicMax) dynamicMax = level * attack;
    else dynamicMax *= decay;
    if (dynamicMax < 500) dynamicMax = 500;

    // Bereichsgrenzen für die LEDs
    float step = dynamicMax / 9.0;

    for (int i = 0; i < 9; i++) {
        float lower = step * i;
        float upper = step * (i + 1);

        uint8_t brightness = 0;
        if (level > lower) {
            brightness = map(level, lower, upper, 0, 255);
            if (brightness > 255) brightness = 255;
            if (brightness < 0) brightness = 0;
        }
        setLedPWM(ledPins[ledOrder[i]], brightness, ledOrder[i]);
    }
}

// =======================================================
// === Nebelmaschine ===
struct FogMachine 
{
  bool active;
  unsigned long startTime;
  unsigned long duration;
  unsigned long lastTrigger;
  unsigned long minInterval;
};

FogMachine fog = {false, 0, 0, 0, 2000}; // minInterval = 2 Sekunden

void triggerFog(unsigned long dur_ms) {
  if (millis() - fog.lastTrigger < fog.minInterval) return;
  fog.active = true;
  fog.startTime = millis();
  fog.duration = dur_ms;
  fog.lastTrigger = millis();
  Serial.println("💨 Nebelmaschine aktiviert für " + String(dur_ms) + " ms");
}

void updateFogMachine() 
{
  if (fog.active == true)
  {
    digitalWrite(FOG_PIN, HIGH); //set FOG_PIN HIGH
  }
  if (fog.active && millis() - fog.startTime >= fog.duration) {
    fog.active = false;
    digitalWrite(FOG_PIN, LOW); //set FOG_PIN LOW
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
// === Firmware Update ===
const char* fwUrl = "https://github.com/Hannes1007/AlexaBridge/releases/latest/download/firmware.bin";
const char* fwVersion = "1.0.2"; // <--- Deine aktuelle Firmware-Version

void checkForUpdates() {
    HTTPClient http;
    http.begin("https://raw.githubusercontent.com/Hannes1007/AlexaBridge/master/version.txt");
    int httpCode = http.GET();
    if (httpCode == 200) {
        String newVersion = http.getString();
        newVersion.trim();
        if (newVersion != fwVersion) {
            Serial.println("Neue Firmware-Version gefunden, Update wird durchgeführt...");
            WiFiClient client;
            t_httpUpdate_return ret = httpUpdate.update(client, fwUrl);
            switch (ret) {
                case HTTP_UPDATE_FAILED:
                    Serial.printf("Update fehlgeschlagen. Fehler (%d): %s\n", 
                        httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
                    break;
                case HTTP_UPDATE_NO_UPDATES:
                    Serial.println("Keine Updates gefunden.");
                    break;
                case HTTP_UPDATE_OK:
                    Serial.println("Update erfolgreich!");
                    break;
            }
        } else {
            Serial.println("Firmware ist aktuell.");
        }
    }
    http.end();
}

// --- Peak-Detection & Trigger-Limitierung ---
const int RMS_HISTORY_SECONDS = 60;
float rmsHistory[RMS_HISTORY_SECONDS] = {0};
unsigned long rmsIndex = 0;

unsigned long fogTriggerTimes[2] = {0, 0}; // Zeitpunkte der letzten 2 Trigger

void addRmsSample(float rms) {
    rmsHistory[rmsIndex] = rms;
    rmsIndex = (rmsIndex + 1) % RMS_HISTORY_SECONDS;
}

float getRmsPeak() {
    float peak = 0;
    for (int i = 0; i < RMS_HISTORY_SECONDS; i++) {
        if (rmsHistory[i] > peak) peak = rmsHistory[i];
    }
    return peak;
}

bool canTriggerFog() {
    unsigned long now = millis();
    int count = 0;
    for (int i = 0; i < 2; i++) {
        if (now - fogTriggerTimes[i] < 60000) count++;
    }
    return count < 2;
}

void registerFogTrigger() {
    // Ältesten Eintrag überschreiben
    if (fogTriggerTimes[0] < fogTriggerTimes[1]) {
        fogTriggerTimes[0] = millis();
    } else {
        fogTriggerTimes[1] = millis();
    }
}

// =======================================================
// === Setup ===
void setup() {
  Serial.begin(115200);
  pinMode(WIFI_RESET_PIN, INPUT);

  // Alle LED-Pins als Output setzen
  pinMode(RL_SCHLUSS_LINKS, OUTPUT);
  pinMode(RL_BREMS_LINKS, OUTPUT);
  pinMode(RL_BLINKER_LINKS, OUTPUT);
  pinMode(RL_RUECKFAHR_LINKS, OUTPUT);

  pinMode(RL_SCHLUSS_RECHTS, OUTPUT);
  pinMode(RL_BREMS_RECHTS, OUTPUT);
  pinMode(RL_BLINKER_RECHTS, OUTPUT);
  pinMode(RL_RUECKFAHR_RECHTS, OUTPUT);
  pinMode(RL_NEBEL_RECHTS, OUTPUT);
  
  // Nebelmschine-Pin als Output setzen
  pinMode(FOG_PIN, OUTPUT);

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

  fauxmo.addDevice(ID_SCHLUSS_RECHTS);
  fauxmo.addDevice(ID_BREMS_RECHTS);
  fauxmo.addDevice(ID_BLINKER_RECHTS);
  fauxmo.addDevice(ID_RUECK_RECHTS);
  fauxmo.addDevice(ID_NEBEL_RECHTS);

  fauxmo.addDevice(ID_PARTY_MODE);

  fauxmo.setPort(80);
  fauxmo.enable(true);

  // Party-Modus direkt aktivieren
  partyModeActive = false;
  fauxmo.setState(ID_PARTY_MODE, false, 0);
  Serial.println("🎉 Party-Modus standardmäßig AUS beim Start");

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

    else if (strcmp(device_name, ID_SCHLUSS_RECHTS) == 0) rlSchlussRechtsControl(state ? value : 0);
    else if (strcmp(device_name, ID_BREMS_RECHTS) == 0)   rlBremsRechtsControl(state ? value : 0);
    else if (strcmp(device_name, ID_BLINKER_RECHTS) == 0) rlBlinkerRechtsControl(state ? value : 0);
    else if (strcmp(device_name, ID_RUECK_RECHTS) == 0)   rlRueckfahrRechtsControl(state ? value : 0);
    else if (strcmp(device_name, ID_NEBEL_RECHTS) == 0)   rlNebelRechtsControl(state ? value : 0);

  });

  // Nach erfolgreichem WLAN-Connect:
  checkForUpdates();
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

      addRmsSample(rms);

      if (partyModeActive) {
        lightOrgan(rms); // Lichtorgel

        // Dynamischer Schwellenwert: 90% des Maximums der letzten 60s
        float peak = getRmsPeak();
        float threshold = peak * 0.9;

        // Mindestgrenze für RMS (z.B. 100, anpassen nach Bedarf)
        const float minThreshold = 100.0;
        Serial.println(rms);

        if (rms > threshold && rms > minThreshold && canTriggerFog()) {
          unsigned long fogDuration = random(500, 1201); // 0,5–1,2s
          triggerFog(fogDuration);
          registerFogTrigger();
        }
      }
    }
  }
#endif

  updateFogMachine(); // immer aufrufen
  fauxmo.handle();
}
