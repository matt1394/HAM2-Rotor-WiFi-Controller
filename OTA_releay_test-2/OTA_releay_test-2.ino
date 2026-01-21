#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>

// ======== Wi-Fi (EDIT THESE) ========
const char* WIFI_SSID = "Punky";
const char* WIFI_PASS = "Ein$tien1";

// Optional: OTA password (leave "" for none)
const char* OTA_PASSWORD = "";  // e.g. "changeme"

// Hostname you’ll see in Arduino IDE (Tools → Port)
const char* OTA_HOSTNAME = "rotator-control";

// ======== Relay mapping ========
// Relay 1 -> GPIO 32
// Relay 2 -> GPIO 33
// Relay 3 -> GPIO 25
// Relay 4 -> GPIO 26
const int relayPins[] = {32, 33, 25, 26};
const int relayCount = sizeof(relayPins) / sizeof(relayPins[0]);

// Your board worked with this setting in the previous test:
const bool RELAY_ACTIVE_LOW = true;

static inline void relayOff(int pin) {
  digitalWrite(pin, RELAY_ACTIVE_LOW ? HIGH : LOW);
}
static inline void relayOn(int pin) {
  digitalWrite(pin, RELAY_ACTIVE_LOW ? LOW : HIGH);
}

// Force relays to start normally-open (OFF)
void setupRelaysNormallyOpen() {
  for (int i = 0; i < relayCount; i++) {
    int pin = relayPins[i];

    relayOff(pin);        // set output latch to OFF (NO open)
    pinMode(pin, OUTPUT); // start driving the pin
    relayOff(pin);        // enforce OFF again
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // helps OTA reliability
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.printf("Connecting to Wi-Fi: %s\n", WIFI_SSID);
  uint32_t start = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    if (millis() - start > 20000) { // 20s timeout
      Serial.println("\nWi-Fi connect timeout; retrying...");
      WiFi.disconnect(true);
      delay(500);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      start = millis();
    }
  }

  Serial.println("\nWi-Fi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setupOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);

  if (OTA_PASSWORD && OTA_PASSWORD[0] != '\0') {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }

  ArduinoOTA
    .onStart([]() {
      Serial.println("\nOTA Start");
    })
    .onEnd([]() {
      Serial.println("\nOTA End");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("OTA Progress: %u%%\r", (progress * 100) / total);
    })
    .onError([](ota_error_t error) {
      Serial.printf("\nOTA Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  Serial.printf("OTA ready. Hostname: %s\n", OTA_HOSTNAME);
}

void setupRelays() {
  Serial.println("Initializing relays OFF...");
  for (int i = 0; i < relayCount; i++) {
    pinMode(relayPins[i], OUTPUT);
    relayOff(relayPins[i]);
    Serial.printf("Relay %d (GPIO %d) OFF\n", i + 1, relayPins[i]);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== EEgo ESP32 Relay Test + OTA ===");

  setupRelays();
  connectWiFi();
  
  if (!MDNS.begin("rotor-control")) {
  Serial.println("Error starting mDNS");
} else {
  Serial.println("mDNS started: rotor-control.local");
}

  setupOTA();
}

void loop() {
  // OTA handler must run frequently
  ArduinoOTA.handle();

  // Relay test pattern (non-blocking-ish: short delays only)
  for (int i = 0; i < relayCount; i++) {
    ArduinoOTA.handle();
    Serial.printf("Relay %d ON\n", i + 1);
    relayOn(relayPins[i]);

    uint32_t t0 = millis();
    while (millis() - t0 < 1200) ArduinoOTA.handle();

    ArduinoOTA.handle();
    Serial.printf("Relay %d OFF\n", i + 1);
    relayOff(relayPins[i]);

    t0 = millis();
    while (millis() - t0 < 600) ArduinoOTA.handle();
  }

  Serial.println("Cycle complete\n");
  uint32_t t0 = millis();
  while (millis() - t0 < 1500) ArduinoOTA.handle();
}
