/*
 * SMART METALLURGY WATCH - ESP32 firmware (Arduino framework, ESP32 core 2.0.x)
 * ---------------------------------------------------------------------------
 * Features
 *  - Digital clock (NTP synced, 7-segment style)
 *  - Wi-Fi to company network, MQTT for live data + call signalling
 *  - Industrial data view (furnace temp/state, ladle, pour status, CO gas)
 *  - Daily check-in / check-out for the pouring unit
 *  - Melt-time logging per casting product (stopwatch vs nominal time)
 *  - All records written as XML, queued in flash, uploaded to cloud path via HTTPS PUT
 *  - Safety: heart-beat monitor, safe-distance from pouring unit (BLE beacon RSSI),
 *    gas alarm, remote evacuation alert, SOS button, melt overtime, low battery
 *  - Intercom (push-to-talk voice over UDP, signalled over MQTT)
 *  - Battery management (LiPo %, charging, power-save, critical shutdown)
 *
 * Hardware (assumed)
 *  ESP32 DevKit | ST7789 240x240 TFT (configure pins in TFT_eSPI User_Setup.h)
 *  MAX30102/MAX30105 (I2C 21/22) | INMP441 I2S mic | MAX98357A I2S amp
 *  3 push buttons | vibration motor (via transistor) | active buzzer | 1S LiPo + TP4056
 *  Plant side: BLE beacon on pouring unit advertising the name BEACON_NAME
 *
 * Libraries: TFT_eSPI, SparkFun MAX3010x, PubSubClient, ArduinoJson (v6)
 *
 * NOTE: prototype firmware. It is NOT a certified safety device. BLE RSSI distance is
 * approximate; use UWB / certified interlocks for life-safety functions and test on site.
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <time.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <driver/i2s.h>
#include <esp_sleep.h>

// ============================== CONFIGURATION ==============================
#define WIFI_SSID       "PLANT_NET"
#define WIFI_PASS       "change-me"
#define EMP_ID          "EMP1042"
#define EMP_NAME        "R. Patil"
#define PLANT_ID        "KOP-FOUNDRY-01"
#define POUR_UNIT_ID    "PU-01"
#define WATCH_ID        "SMW-0007"

#define CLOUD_BASE      "https://cloud.company.com/smartwatch"   // XML cloud path root
#define CLOUD_TOKEN     "REPLACE_WITH_API_TOKEN"
static const char* ROOT_CA = "";   // paste PEM root CA; empty = insecure (testing only)

#define MQTT_HOST       "mqtt.company.local"
#define MQTT_PORT       1883
#define TOPIC_DATA      "plant/metallurgy/" POUR_UNIT_ID "/data"
#define TOPIC_ALERT     "plant/metallurgy/alert"
#define TOPIC_CALL_PFX  "plant/call/"
#define AUDIO_PORT      4210

#define TZ_OFFSET_SEC   19800      // IST (UTC+5:30)
#define BEACON_NAME     "POUR-PU-01"
#define BLE_TX_POWER_1M (-59)      // calibrate RSSI at 1 m
#define BLE_PATH_LOSS_N 2.5f

// Safety thresholds
#define SAFE_DIST_M         5.0f
#define DANGER_DIST_M       3.0f
#define SAFE_DIST_POUR_M    8.0f   // larger zone while pouring is active
#define DANGER_DIST_POUR_M  5.0f
#define HR_HIGH_BPM         150
#define HR_LOW_BPM          45
#define HR_SUSTAIN_MS       10000
#define CO_WARN_PPM         35
#define CO_DANGER_PPM       200

// Pins
#define PIN_NEXT 32
#define PIN_OK   33
#define PIN_PTT  25
#define PIN_VIB  26
#define PIN_BUZ  27
#define PIN_BAT  34   // ADC via 1:2 divider
#define PIN_CHRG 35   // TP4056 CHRG (active low)
#define PIN_BL   5    // TFT backlight PWM
#define I2S_MIC_BCK 14
#define I2S_MIC_WS  15
#define I2S_MIC_DIN 13
#define I2S_SPK_BCK 18
#define I2S_SPK_WS  19
#define I2S_SPK_DOUT 4

// ============================== TYPES / STATE ==============================
enum Screen { S_CLOCK, S_DATA, S_MELT, S_ATTEND, S_CALL, S_SAFETY, S_COUNT };
enum AlertCode { A_HR_HIGH, A_HR_LOW, A_DIST, A_GAS, A_BATT, A_REMOTE, A_MELT, A_SOS, A_COUNT };
enum CallState { C_IDLE, C_RING_OUT, C_RING_IN, C_ACTIVE };

struct Product { const char* name; uint16_t nominalMin; };
// Nominal melt times are placeholders - calibrate for your furnace.
Product products[] = {
  {"Aluminium A356", 45}, {"Gray Cast Iron", 90}, {"Ductile Iron", 110},
  {"Carbon Steel", 120}, {"Brass", 60}, {"Bronze", 70}, {"Copper", 80}
};
const int NPROD = sizeof(products) / sizeof(products[0]);

struct Contact { const char* name; const char* id; };
Contact contacts[] = {{"Control Room", "CTRL"}, {"Pouring Unit", "PU-01"},
                      {"Supervisor", "SUP01"}, {"Safety Officer", "SAFE1"}};
const int NCONT = sizeof(contacts) / sizeof(contacts[0]);

struct Btn {
  uint8_t pin; bool down; uint32_t t0; bool longFired;
  Btn(uint8_t p) : pin(p), down(false), t0(0), longFired(false) {}
};
// returns 0 none, 1 short press (on release), 2 long press (fires while held)
int pollBtn(Btn& b, uint32_t longMs = 800) {
  bool p = digitalRead(b.pin) == LOW; uint32_t now = millis();
  if (p && !b.down) { b.down = true; b.t0 = now; b.longFired = false; }
  else if (p && b.down && !b.longFired && now - b.t0 >= longMs) { b.longFired = true; return 2; }
  else if (!p && b.down) { b.down = false; if (!b.longFired && now - b.t0 > 30) return 1; }
  return 0;
}

TFT_eSPI tft;
MAX30105 ppg;
WiFiClient mqttNet;
PubSubClient mqtt(mqttNet);
WiFiUDP udp;
Preferences prefs;
Btn bNext(PIN_NEXT), bOk(PIN_OK), bPtt(PIN_PTT);

Screen screen = S_CLOCK;
bool needFull = true;
uint32_t lastInput = 0, lastDraw = 0;

// live data
volatile int hr = 0;                  // bpm, 0 = no signal
volatile uint32_t lastHrMs = 0;
volatile float distM = -1;            // metres, -1 = beacon not seen
volatile uint32_t beaconSeenMs = 0;
float furnaceTemp = 0; String furnaceState = "--", ladleState = "--"; int coPpm = 0;
volatile bool pourActive = false;
uint32_t lastDataMs = 0;

// battery
float battV = 4.0f; int battPct = 100; bool charging = false, lowPower = false;

// alerts
String alertMsg; uint8_t alertLevel = 0; uint32_t alertUntil = 0;
uint32_t alertLast[A_COUNT] = {0};

// attendance + melt
bool checkedIn = false; time_t checkInEpoch = 0;
bool melting = false; time_t meltStartEpoch = 0; uint32_t meltStartMs = 0; int prodIdx = 0;
int meltsToday = 0; uint32_t meltSecToday = 0; int lastYday = -1;
bool meltOvertimeFlagged = false;

// call
CallState callState = C_IDLE; int contactIdx = 0;
String callPeerId, callPeerName; IPAddress peerIP; uint32_t callT0 = 0, callStartMs = 0;

// storage
uint16_t seqNo = 0; int qCount = 0;
uint32_t lastTelemetry = 0, lastFlush = 0, lastWifiTry = 0, lastMqttTry = 0, lastBatt = 0;

// ============================== XML + CLOUD ==============================
String xmlEsc(const String& s) {
  String o; o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') o += "&amp;"; else if (c == '<') o += "&lt;"; else if (c == '>') o += "&gt;";
    else if (c == '"') o += "&quot;"; else if (c == '\'') o += "&apos;"; else o += c;
  }
  return o;
}
bool timeValid() { return time(nullptr) > 1700000000; }
String fmtTime(time_t t, const char* f) { struct tm tm; localtime_r(&t, &tm); char b[40]; strftime(b, sizeof b, f, &tm); return String(b); }

// Build one XML record and queue it in flash. Uploaded later to CLOUD_BASE/<plant>/<emp>/<date>/<file>.xml
void queueXml(const char* type, const String& body) {
  if (qCount > 300 && strcmp(type, "TELEMETRY") == 0) return;   // protect flash
  time_t now = time(nullptr);
  String fn = "/q/" + fmtTime(now, "%Y%m%d_%H%M%S") + "_" + String(seqNo++) + "_" + type + ".xml";
  String x = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  x += "<WatchRecord type=\"" + String(type) + "\" plant=\"" PLANT_ID "\" employee=\"" EMP_ID "\" watch=\"" WATCH_ID "\">\n";
  x += "  <EmployeeName>" + xmlEsc(EMP_NAME) + "</EmployeeName>\n";
  x += "  <Timestamp>" + fmtTime(now, "%Y-%m-%dT%H:%M:%S") + "</Timestamp>\n";
  x += body;
  x += "</WatchRecord>\n";
  File f = LittleFS.open(fn, "w");
  if (f) { f.print(x); f.close(); qCount++; }
}

void flushQueue() {
  if (WiFi.status() != WL_CONNECTED || !timeValid()) return;
  File dir = LittleFS.open("/q");
  if (!dir) return;
  String names[3]; int n = 0;
  for (File f = dir.openNextFile(); f && n < 3; f = dir.openNextFile()) {
    String nm = f.name(); int sl = nm.lastIndexOf('/'); if (sl >= 0) nm = nm.substring(sl + 1);
    names[n++] = nm;
  }
  dir.close();
  for (int i = 0; i < n; i++) {
    String path = "/q/" + names[i];
    File f = LittleFS.open(path, "r"); if (!f) continue;
    String xml = f.readString(); f.close();
    String url = String(CLOUD_BASE) + "/" PLANT_ID "/" EMP_ID "/" + names[i].substring(0, 8) + "/" + names[i];
    WiFiClientSecure sc;
    if (strlen(ROOT_CA)) sc.setCACert(ROOT_CA); else sc.setInsecure();
    HTTPClient http;
    if (!http.begin(sc, url)) continue;
    http.addHeader("Content-Type", "application/xml");
    http.addHeader("Authorization", "Bearer " CLOUD_TOKEN);
    http.setTimeout(6000);
    int code = http.PUT(xml);
    http.end();
    if (code >= 200 && code < 300) { LittleFS.remove(path); if (qCount > 0) qCount--; }
    else break;   // retry next cycle
  }
}

void countQueue() {
  qCount = 0; File dir = LittleFS.open("/q");
  if (!dir) return;
  for (File f = dir.openNextFile(); f; f = dir.openNextFile()) qCount++;
}

// ============================== ALERTS ==============================
// lvl: 1 = warning, 2 = danger
void raiseAlert(AlertCode code, const char* name, const String& msg, uint8_t lvl) {
  uint32_t now = millis();
  if (code != A_SOS && alertLast[code] && now - alertLast[code] < 30000) return;
  alertLast[code] = now;
  alertMsg = msg; alertLevel = lvl; alertUntil = now + (lvl >= 2 ? 15000 : 8000);
  needFull = true;
  String b = "  <Alert code=\"" + String(name) + "\" level=\"" + (lvl >= 2 ? "DANGER" : "WARNING") + "\">" + xmlEsc(msg) + "</Alert>\n";
  b += "  <HeartRateBpm>" + String((int)hr) + "</HeartRateBpm>\n  <DistanceToPourUnitM>" + String(distM, 1) + "</DistanceToPourUnitM>\n";
  queueXml("ALERT", b);
  if (mqtt.connected()) mqtt.publish("plant/alerts/" EMP_ID, (String(name) + "|" + msg).c_str());
  lastFlush = 0;   // upload alerts immediately
}

void alertOutputs() {
  uint32_t now = millis(); bool on = false;
  if (now < alertUntil) {
    if (alertLevel >= 2) on = (now / 150) % 2 == 0;      // fast pulses
    else on = (now % 1000) < 120;                         // single short pulse/s
  }
  digitalWrite(PIN_VIB, on); digitalWrite(PIN_BUZ, on || (callState == C_RING_IN && (now / 400) % 2 == 0));
}

// ============================== BATTERY ==============================
int battPercent(float v) {
  static const float V[] = {3.27, 3.61, 3.69, 3.73, 3.77, 3.80, 3.84, 3.87, 3.91, 3.95, 4.02, 4.11, 4.20};
  static const int   P[] = {0, 5, 10, 20, 30, 40, 50, 60, 65, 70, 80, 90, 100};
  if (v <= V[0]) return 0; if (v >= V[12]) return 100;
  for (int i = 1; i < 13; i++) if (v <= V[i]) return P[i - 1] + (int)((v - V[i - 1]) * (P[i] - P[i - 1]) / (V[i] - V[i - 1]));
  return 100;
}

void setBrightness(uint8_t b) { ledcWrite(0, b); }

void batteryTask() {
  uint32_t acc = 0; for (int i = 0; i < 16; i++) acc += analogReadMilliVolts(PIN_BAT);
  float v = (acc / 16.0f) * 2.0f / 1000.0f;
  battV = battV * 0.8f + v * 0.2f;
  charging = digitalRead(PIN_CHRG) == LOW;
  battPct = battPercent(battV);
  bool lp = (!charging && battPct < 20);
  if (lp != lowPower) { lowPower = lp; needFull = true; }
  if (!charging && battPct < 15) raiseAlert(A_BATT, "LOW_BATTERY", "Battery " + String(battPct) + "% - charge now", 1);
  if (!charging && battPct <= 3) {                         // critical: log, flush, sleep
    queueXml("BATTERY", "  <Battery percent=\"" + String(battPct) + "\" volts=\"" + String(battV, 2) + "\" event=\"SHUTDOWN\"/>\n");
    uint32_t t = millis(); while (millis() - t < 4000) { flushQueue(); delay(200); }
    tft.fillScreen(TFT_BLACK); setBrightness(0);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0);
    esp_deep_sleep_start();
  }
}

// ============================== SENSOR / BLE TASKS ==============================
void sensorTask(void*) {
  float rates[4] = {0}; int ri = 0, filled = 0; uint32_t lastBeat = 0;
  for (;;) {
    long ir = ppg.getIR();
    if (ir < 30000) { if (millis() - lastHrMs > 5000) hr = 0; }
    else if (checkForBeat(ir)) {
      uint32_t now = millis(); uint32_t d = now - lastBeat; lastBeat = now;
      float b = 60000.0f / d;
      if (b > 30 && b < 220) {
        rates[ri++ % 4] = b; if (filled < 4) filled++;
        float s = 0; for (int i = 0; i < filled; i++) s += rates[i];
        hr = (int)(s / filled); lastHrMs = now;
      }
    }
    if (millis() - lastHrMs > 8000) hr = 0;
    vTaskDelay(lowPower ? 20 : 10);
  }
}

void bleTask(void*) {
  BLEDevice::init("");
  BLEScan* scan = BLEDevice::getScan();
  scan->setActiveScan(true); scan->setInterval(100); scan->setWindow(99);
  static float ema = 0; static bool init = false;
  for (;;) {
    BLEScanResults r = scan->start(1, false);
    for (int i = 0; i < r.getCount(); i++) {
      BLEAdvertisedDevice d = r.getDevice(i);
      if (d.haveName() && d.getName() == BEACON_NAME) {
        float rssi = d.getRSSI();
        ema = init ? ema * 0.7f + rssi * 0.3f : rssi; init = true;
        distM = powf(10.0f, (BLE_TX_POWER_1M - ema) / (10.0f * BLE_PATH_LOSS_N));
        beaconSeenMs = millis();
      }
    }
    scan->clearResults();
    if (millis() - beaconSeenMs > 10000) distM = -1;
    vTaskDelay(lowPower ? 3000 : 500);
  }
}

// ============================== INTERCOM ==============================
String myIp() { return WiFi.localIP().toString(); }
String callTopic(const String& id) { return String(TOPIC_CALL_PFX) + id; }

String part(const String& s, int idx) {
  int start = 0;
  for (int i = 0; i < idx; i++) { start = s.indexOf('|', start); if (start < 0) return ""; start++; }
  int end = s.indexOf('|', start); return end < 0 ? s.substring(start) : s.substring(start, end);
}

void endCall(bool notify) {
  if (callState == C_IDLE) return;
  if (notify && mqtt.connected()) mqtt.publish(callTopic(callPeerId).c_str(), "HANGUP|" EMP_ID);
  if (callState == C_ACTIVE) {
    uint32_t dur = (millis() - callStartMs) / 1000;
    queueXml("CALL", "  <Call peer=\"" + xmlEsc(callPeerId) + "\" durationSec=\"" + String(dur) + "\"/>\n");
  }
  callState = C_IDLE; needFull = true;
}

void handleCallMsg(const String& m) {
  String cmd = part(m, 0);
  if (cmd == "RING" && callState == C_IDLE) {
    callPeerId = part(m, 1); callPeerName = part(m, 2); peerIP.fromString(part(m, 3));
    callState = C_RING_IN; callT0 = millis(); screen = S_CALL; needFull = true;
  } else if (cmd == "ACCEPT" && callState == C_RING_OUT) {
    peerIP.fromString(part(m, 2)); callState = C_ACTIVE; callStartMs = millis(); needFull = true;
  } else if (cmd == "HANGUP" && callState != C_IDLE) {
    endCall(false);
  }
}

void startCall() {
  callPeerId = contacts[contactIdx].id; callPeerName = contacts[contactIdx].name;
  if (!mqtt.connected()) return;
  mqtt.publish(callTopic(callPeerId).c_str(), ("RING|" EMP_ID "|" EMP_NAME "|" + myIp()).c_str());
  callState = C_RING_OUT; callT0 = millis(); needFull = true;
}

void acceptCall() {
  mqtt.publish(callTopic(callPeerId).c_str(), ("ACCEPT|" EMP_ID "|" + myIp()).c_str());
  callState = C_ACTIVE; callStartMs = millis(); needFull = true;
}

void i2sInit() {
  i2s_config_t mic = {};
  mic.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX); mic.sample_rate = 16000;
  mic.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT; mic.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  mic.communication_format = I2S_COMM_FORMAT_STAND_I2S; mic.dma_buf_count = 4; mic.dma_buf_len = 256;
  mic.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  i2s_pin_config_t mp = {I2S_MIC_BCK, I2S_MIC_WS, I2S_PIN_NO_CHANGE, I2S_MIC_DIN};
  i2s_driver_install(I2S_NUM_0, &mic, 0, NULL); i2s_set_pin(I2S_NUM_0, &mp);

  i2s_config_t spk = mic;
  spk.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX); spk.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  spk.tx_desc_auto_clear = true;
  i2s_pin_config_t sp = {I2S_SPK_BCK, I2S_SPK_WS, I2S_SPK_DOUT, I2S_PIN_NO_CHANGE};
  i2s_driver_install(I2S_NUM_1, &spk, 0, NULL); i2s_set_pin(I2S_NUM_1, &sp);
}

void audioTask(void*) {
  static int32_t raw[256]; static int16_t pcm[256]; size_t n;
  for (;;) {
    if (callState == C_ACTIVE) {
      if (digitalRead(PIN_PTT) == LOW) {                     // push-to-talk: transmit
        i2s_read(I2S_NUM_0, raw, sizeof raw, &n, 50 / portTICK_PERIOD_MS);
        int cnt = n / 4;
        for (int i = 0; i < cnt; i++) pcm[i] = (int16_t)(raw[i] >> 14);
        udp.beginPacket(peerIP, AUDIO_PORT); udp.write((uint8_t*)pcm, cnt * 2); udp.endPacket();
      }
      int sz = udp.parsePacket();
      if (sz > 0 && sz <= (int)sizeof pcm) {
        udp.read((uint8_t*)pcm, sz); i2s_write(I2S_NUM_1, pcm, sz, &n, 20 / portTICK_PERIOD_MS);
      }
      vTaskDelay(1);
    } else vTaskDelay(50);
  }
}

// ============================== MQTT ==============================
void onMqtt(char* topic, byte* pl, unsigned int len) {
  String t = topic, m; m.reserve(len);
  for (unsigned i = 0; i < len; i++) m += (char)pl[i];
  if (t == TOPIC_DATA) {                       // {"temp":1480,"state":"MELTING","ladle":"READY","pour":false,"co":12}
    StaticJsonDocument<320> d;
    if (deserializeJson(d, m) == DeserializationError::Ok) {
      furnaceTemp = d["temp"] | 0.0f; furnaceState = String((const char*)(d["state"] | "--"));
      ladleState = String((const char*)(d["ladle"] | "--")); pourActive = d["pour"] | false;
      coPpm = d["co"] | 0; lastDataMs = millis();
    }
  } else if (t == TOPIC_ALERT) {
    bool evac = m.startsWith("EVAC");
    raiseAlert(A_REMOTE, "PLANT_ALERT", m, evac ? 2 : 1);
  } else if (t == callTopic(EMP_ID)) handleCallMsg(m);
}

void mqttLoop() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!mqtt.connected()) {
    if (millis() - lastMqttTry < 5000) return;
    lastMqttTry = millis();
    if (mqtt.connect(WATCH_ID, NULL, NULL)) {
      mqtt.subscribe(TOPIC_DATA); mqtt.subscribe(TOPIC_ALERT); mqtt.subscribe(callTopic(EMP_ID).c_str());
    }
    return;
  }
  mqtt.loop();
}

// ============================== ATTENDANCE / MELT ==============================
void dayRollover() {
  time_t now = time(nullptr); struct tm tm; localtime_r(&now, &tm);
  if (lastYday != tm.tm_yday) { if (lastYday != -1) { meltsToday = 0; meltSecToday = 0; } lastYday = tm.tm_yday; }
}

void toggleAttendance() {
  time_t now = time(nullptr);
  String unit = "  <Unit>" POUR_UNIT_ID "</Unit>\n";
  if (!checkedIn) {
    checkedIn = true; checkInEpoch = now;
    queueXml("CHECKIN", unit + "  <DistanceToPourUnitM>" + String(distM, 1) + "</DistanceToPourUnitM>\n");
  } else {
    double hrs = difftime(now, checkInEpoch) / 3600.0;
    checkedIn = false;
    queueXml("CHECKOUT", unit + "  <HoursOnDuty>" + String(hrs, 2) + "</HoursOnDuty>\n  <MeltCycles>" + String(meltsToday) +
             "</MeltCycles>\n  <TotalMeltMinutes>" + String(meltSecToday / 60) + "</TotalMeltMinutes>\n");
  }
  prefs.putBool("in", checkedIn); prefs.putULong("inT", (unsigned long)checkInEpoch);
  needFull = true; lastFlush = 0;
}

void toggleMelt() {
  if (!checkedIn) { raiseAlert(A_MELT, "NOT_CHECKED_IN", "Check in first", 1); return; }
  time_t now = time(nullptr);
  if (!melting) {
    melting = true; meltStartEpoch = now; meltStartMs = millis(); meltOvertimeFlagged = false;
  } else {
    melting = false;
    uint32_t sec = (millis() - meltStartMs) / 1000;
    int nominal = products[prodIdx].nominalMin;
    float var = nominal ? ((sec / 60.0f) - nominal) * 100.0f / nominal : 0;
    meltsToday++; meltSecToday += sec;
    String b = "  <Unit>" POUR_UNIT_ID "</Unit>\n  <Melt product=\"" + xmlEsc(products[prodIdx].name) + "\">\n";
    b += "    <Start>" + fmtTime(meltStartEpoch, "%Y-%m-%dT%H:%M:%S") + "</Start>\n";
    b += "    <End>" + fmtTime(now, "%Y-%m-%dT%H:%M:%S") + "</End>\n";
    b += "    <DurationSec>" + String(sec) + "</DurationSec>\n    <NominalMin>" + String(nominal) + "</NominalMin>\n";
    b += "    <VariancePercent>" + String(var, 1) + "</VariancePercent>\n";
    b += "    <FurnaceTempC>" + String(furnaceTemp, 0) + "</FurnaceTempC>\n  </Melt>\n";
    queueXml("MELT", b); lastFlush = 0;
  }
  needFull = true;
}

// ============================== SAFETY MONITOR ==============================
void safetyCheck() {
  uint32_t now = millis();
  // distance from pouring unit
  float d = distM;
  if (d >= 0) {
    float safe = pourActive ? SAFE_DIST_POUR_M : SAFE_DIST_M;
    float dang = pourActive ? DANGER_DIST_POUR_M : DANGER_DIST_M;
    if (d < dang) raiseAlert(A_DIST, "DISTANCE", "TOO CLOSE to pour unit " + String(d, 1) + "m", 2);
    else if (d < safe) raiseAlert(A_DIST, "DISTANCE", "Keep back: " + String(d, 1) + "m (min " + String(safe, 0) + "m)", 1);
  }
  // heart beat (sustained)
  static uint32_t hiSince = 0, loSince = 0; int h = hr;
  if (h > HR_HIGH_BPM) { if (!hiSince) hiSince = now; if (now - hiSince > HR_SUSTAIN_MS) raiseAlert(A_HR_HIGH, "HEART_RATE_HIGH", "High heart rate " + String(h) + " bpm - rest & hydrate", 2); }
  else hiSince = 0;
  if (h > 0 && h < HR_LOW_BPM) { if (!loSince) loSince = now; if (now - loSince > HR_SUSTAIN_MS) raiseAlert(A_HR_LOW, "HEART_RATE_LOW", "Low heart rate " + String(h) + " bpm", 2); }
  else loSince = 0;
  // gas
  if (millis() - lastDataMs < 15000) {
    if (coPpm >= CO_DANGER_PPM) raiseAlert(A_GAS, "CO_GAS", "CO " + String(coPpm) + " ppm - EVACUATE", 2);
    else if (coPpm >= CO_WARN_PPM) raiseAlert(A_GAS, "CO_GAS", "CO " + String(coPpm) + " ppm - ventilate", 1);
  }
  // melt overtime (+20 %)
  if (melting && !meltOvertimeFlagged && (millis() - meltStartMs) / 60000.0f > products[prodIdx].nominalMin * 1.2f) {
    meltOvertimeFlagged = true;
    raiseAlert(A_MELT, "MELT_OVERTIME", String(products[prodIdx].name) + " melt overtime", 1);
  }
}

// ============================== UI ==============================
void row(int y, int h, const String& s, uint16_t col, uint8_t font, int x = 8) {
  tft.fillRect(0, y, 240, h, TFT_BLACK); tft.setTextColor(col, TFT_BLACK);
  tft.setTextDatum(TL_DATUM); tft.drawString(s, x, y, font);
}

void drawStatusBar() {
  time_t now = time(nullptr);
  String t = timeValid() ? fmtTime(now, "%H:%M") : "--:--";
  tft.fillRect(0, 0, 240, 22, 0x2104);
  tft.setTextDatum(TL_DATUM); tft.setTextColor(WiFi.status() == WL_CONNECTED ? TFT_GREEN : TFT_RED, 0x2104); tft.drawString("W", 6, 3, 2);
  tft.setTextColor(mqtt.connected() ? TFT_GREEN : TFT_RED, 0x2104); tft.drawString("C", 22, 3, 2);
  tft.setTextColor(TFT_WHITE, 0x2104); tft.drawString(t, 90, 3, 2);
  uint16_t bc = charging ? TFT_CYAN : (battPct < 20 ? TFT_RED : TFT_GREEN);
  tft.setTextColor(bc, 0x2104); tft.setTextDatum(TR_DATUM);
  tft.drawString(charging ? "CHG " + String(battPct) + "%" : String(battPct) + "%", 234, 3, 2);
  tft.setTextDatum(TL_DATUM);
}

void drawAlertOverlay() {
  bool flash = (millis() / 300) % 2 == 0;
  uint16_t bg = alertLevel >= 2 ? (flash ? TFT_RED : 0x8000) : (flash ? TFT_ORANGE : 0x8400);
  tft.fillRect(0, 22, 240, 218, bg); tft.setTextColor(TFT_WHITE, bg); tft.setTextDatum(MC_DATUM);
  tft.drawString(alertLevel >= 2 ? "DANGER" : "WARNING", 120, 60, 4);
  tft.setTextDatum(TL_DATUM);
  tft.setTextWrap(true); tft.setCursor(10, 100); tft.setTextFont(2); tft.print(alertMsg);
}

void drawScreen() {
  drawStatusBar();
  time_t now = time(nullptr);
  switch (screen) {
    case S_CLOCK:
      tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.setTextDatum(MC_DATUM);
      tft.drawString(timeValid() ? fmtTime(now, "%H:%M") : "--:--", 120, 90, 7);
      tft.drawString(timeValid() ? fmtTime(now, ":%S") : ":--", 120, 140, 4);
      tft.drawString(timeValid() ? fmtTime(now, "%a %d %b %Y") : "Waiting for NTP", 120, 175, 4);
      tft.setTextDatum(TL_DATUM);
      row(205, 22, String(EMP_NAME) + (checkedIn ? "  ON DUTY" : "  OFF DUTY"), checkedIn ? TFT_GREEN : TFT_YELLOW, 2);
      row(224, 16, "HR " + String((int)hr) + "  Dist " + (distM >= 0 ? String(distM, 1) + "m" : "--"), TFT_WHITE, 2);
      break;
    case S_DATA: {
      bool stale = millis() - lastDataMs > 15000;
      row(26, 22, "FURNACE " POUR_UNIT_ID, TFT_YELLOW, 2);
      row(50, 40, stale ? "-- C" : String(furnaceTemp, 0) + " C", stale ? TFT_DARKGREY : TFT_ORANGE, 6);
      row(98, 22, "State : " + furnaceState, TFT_WHITE, 4);
      row(128, 22, "Ladle : " + ladleState, TFT_WHITE, 4);
      row(158, 22, String("Pour  : ") + (pourActive ? "ACTIVE" : "idle"), pourActive ? TFT_RED : TFT_GREEN, 4);
      row(188, 22, "CO    : " + String(coPpm) + " ppm", coPpm >= CO_WARN_PPM ? TFT_RED : TFT_GREEN, 4);
      row(218, 18, stale ? "No data from plant" : "Live", TFT_DARKGREY, 2);
      break; }
    case S_MELT: {
      uint32_t sec = melting ? (millis() - meltStartMs) / 1000 : 0;
      char b[16]; snprintf(b, sizeof b, "%02u:%02u:%02u", sec / 3600, (sec / 60) % 60, sec % 60);
      row(26, 24, "Product (hold OK=next)", TFT_YELLOW, 2);
      row(50, 26, products[prodIdx].name, TFT_WHITE, 4);
      row(82, 22, "Nominal " + String(products[prodIdx].nominalMin) + " min", TFT_CYAN, 2);
      row(110, 40, b, melting ? TFT_ORANGE : TFT_DARKGREY, 6);
      row(160, 22, melting ? "MELTING - OK to stop" : "OK = start melt", melting ? TFT_RED : TFT_GREEN, 2);
      row(184, 20, "Today: " + String(meltsToday) + " melts, " + String(meltSecToday / 60) + " min", TFT_WHITE, 2);
      break; }
    case S_ATTEND:
      row(26, 24, "ATTENDANCE - " POUR_UNIT_ID, TFT_YELLOW, 2);
      row(54, 36, checkedIn ? "CHECKED IN" : "CHECKED OUT", checkedIn ? TFT_GREEN : TFT_RED, 4);
      if (checkedIn) {
        double h = difftime(now, checkInEpoch) / 3600.0;
        row(100, 22, "Since " + fmtTime(checkInEpoch, "%H:%M") + "  (" + String(h, 1) + " h)", TFT_WHITE, 2);
      } else row(100, 22, "", TFT_WHITE, 2);
      row(130, 22, "ID: " EMP_ID, TFT_CYAN, 2);
      row(160, 22, checkedIn ? "HOLD OK to check out" : "HOLD OK to check in", TFT_GREEN, 2);
      row(190, 22, "Queued XML: " + String(qCount), TFT_DARKGREY, 2);
      break;
    case S_CALL:
      row(26, 24, "INTERCOM", TFT_YELLOW, 2);
      if (callState == C_IDLE) {
        row(54, 28, contacts[contactIdx].name, TFT_WHITE, 4);
        row(96, 22, "OK = call   hold OK = next", TFT_GREEN, 2);
        row(120, 22, "", TFT_WHITE, 2);
      } else if (callState == C_RING_OUT) {
        row(54, 28, "Calling " + callPeerName, TFT_WHITE, 4);
        row(96, 22, "OK = cancel", TFT_RED, 2);
      } else if (callState == C_RING_IN) {
        row(54, 28, callPeerName, TFT_WHITE, 4);
        row(96, 22, "INCOMING  OK=answer hold=reject", TFT_ORANGE, 2);
      } else {
        uint32_t s = (millis() - callStartMs) / 1000; char b[8]; snprintf(b, sizeof b, "%02u:%02u", s / 60, s % 60);
        row(54, 28, callPeerName, TFT_GREEN, 4);
        row(96, 36, b, TFT_WHITE, 6);
        row(140, 22, "Hold PTT to talk. OK = end", TFT_CYAN, 2);
      }
      break;
    case S_SAFETY: {
      float safe = pourActive ? SAFE_DIST_POUR_M : SAFE_DIST_M, dang = pourActive ? DANGER_DIST_POUR_M : DANGER_DIST_M;
      float d = distM; uint16_t dc = d < 0 ? TFT_DARKGREY : (d < dang ? TFT_RED : (d < safe ? TFT_ORANGE : TFT_GREEN));
      row(26, 22, "SAFETY", TFT_YELLOW, 2);
      row(50, 22, "Distance to pour unit", TFT_WHITE, 2);
      row(72, 40, d < 0 ? "no beacon" : String(d, 1) + " m", dc, d < 0 ? 4 : 6);
      row(120, 20, "Safe >" + String(safe, 0) + "m  Danger <" + String(dang, 0) + "m", TFT_DARKGREY, 2);
      row(146, 28, "HR: " + (hr > 0 ? String((int)hr) + " bpm" : String("--")), (hr > HR_HIGH_BPM || (hr > 0 && hr < HR_LOW_BPM)) ? TFT_RED : TFT_GREEN, 4);
      row(182, 22, "Batt " + String(battV, 2) + "V  " + String(battPct) + "%", battPct < 20 ? TFT_RED : TFT_GREEN, 2);
      row(204, 22, lowPower ? "POWER SAVE ON" : "Hold PTT 3s = SOS", lowPower ? TFT_ORANGE : TFT_DARKGREY, 2);
      break; }
    default: break;
  }
}

// ============================== INPUT ==============================
void handleInput() {
  int n = pollBtn(bNext), o = pollBtn(bOk), p = 0;
  static uint32_t pttDown = 0;
  p = pollBtn(bPtt, 3000);
  if (n || o || p) { lastInput = millis(); setBrightness(lowPower ? 80 : 220); if (alertLevel < 2) alertUntil = 0; needFull = true; }

  if (n == 1) { screen = (Screen)((screen + 1) % S_COUNT); needFull = true; }
  if (p == 2 && callState != C_ACTIVE) raiseAlert(A_SOS, "SOS", String(EMP_NAME) + " pressed SOS!", 2);

  switch (screen) {
    case S_MELT:   if (o == 1) toggleMelt(); if (o == 2) { if (!melting) prodIdx = (prodIdx + 1) % NPROD; } break;
    case S_ATTEND: if (o == 2) toggleAttendance(); break;
    case S_CALL:
      if (callState == C_IDLE) { if (o == 1) startCall(); if (o == 2) contactIdx = (contactIdx + 1) % NCONT; }
      else if (callState == C_RING_IN) { if (o == 1) acceptCall(); if (o == 2) endCall(true); }
      else if (o == 1) endCall(true);
      break;
    default: break;
  }
}

// ============================== SETUP / LOOP ==============================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_NEXT, INPUT_PULLUP); pinMode(PIN_OK, INPUT_PULLUP); pinMode(PIN_PTT, INPUT_PULLUP);
  pinMode(PIN_VIB, OUTPUT); pinMode(PIN_BUZ, OUTPUT); pinMode(PIN_CHRG, INPUT);
  analogReadResolution(12);
  ledcSetup(0, 5000, 8); ledcAttachPin(PIN_BL, 0); setBrightness(220);

  tft.init(); tft.setRotation(0); tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.drawString("Smart Metallurgy Watch", 10, 100, 2);

  LittleFS.begin(true); LittleFS.mkdir("/q"); countQueue();
  prefs.begin("watch", false);
  checkedIn = prefs.getBool("in", false); checkInEpoch = (time_t)prefs.getULong("inT", 0);

  Wire.begin(21, 22);
  if (ppg.begin(Wire, I2C_SPEED_FAST)) { ppg.setup(); ppg.setPulseAmplitudeRed(0x1F); ppg.setPulseAmplitudeGreen(0); }

  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);
  configTime(TZ_OFFSET_SEC, 0, "pool.ntp.org", "time.google.com");
  mqtt.setServer(MQTT_HOST, MQTT_PORT); mqtt.setCallback(onMqtt); mqtt.setBufferSize(512);
  udp.begin(AUDIO_PORT);
  i2sInit();

  xTaskCreatePinnedToCore(sensorTask, "hr", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(bleTask, "ble", 6144, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(audioTask, "audio", 6144, NULL, 3, NULL, 0);
  batteryTask(); lastInput = millis();
  tft.fillScreen(TFT_BLACK);
}

void loop() {
  uint32_t now = millis();
  handleInput();
  mqttLoop();

  if (WiFi.status() != WL_CONNECTED && now - lastWifiTry > 10000) { lastWifiTry = now; WiFi.reconnect(); }
  if (callState == C_RING_OUT && now - callT0 > 30000) endCall(true);        // no answer
  if (callState == C_RING_IN && now - callT0 > 30000) endCall(true);

  if (now - lastBatt > 5000) { lastBatt = now; batteryTask(); }
  if (timeValid()) dayRollover();
  safetyCheck();
  alertOutputs();

  if (now - lastTelemetry > (lowPower ? 300000UL : 60000UL) && timeValid()) {
    lastTelemetry = now;
    String b = "  <Telemetry>\n    <HeartRateBpm>" + String((int)hr) + "</HeartRateBpm>\n    <DistanceToPourUnitM>" + String(distM, 1) +
               "</DistanceToPourUnitM>\n    <BatteryPercent>" + String(battPct) + "</BatteryPercent>\n    <OnDuty>" + String(checkedIn ? "true" : "false") +
               "</OnDuty>\n    <Melting>" + String(melting ? "true" : "false") + "</Melting>\n  </Telemetry>\n";
    queueXml("TELEMETRY", b);
  }
  if (now - lastFlush > 5000) { lastFlush = now; flushQueue(); }

  // display: dim after 20 s idle (never during alert / call), faster refresh on clock
  if (now - lastInput > 20000 && now > alertUntil && callState == C_IDLE) setBrightness(lowPower ? 20 : 40);
  if (now - lastDraw > (alertUntil > now ? 300 : 500)) {
    lastDraw = now;
    if (needFull) { tft.fillScreen(TFT_BLACK); needFull = false; }
    if (now < alertUntil) drawAlertOverlay(); else drawScreen();
  }
  delay(5);
}
