/*
  ESP32-S3 Water Monitoring Firmware
  Sensors:
  1. Gravity pH Meter V2.0      -> A0
  2. Gravity TDS V1.0           -> A1
  3. DS18B20                    -> GPIO 14
  4. A01NYUB / A01ANYUB V2 UART -> Serial1 RX=18, TX=17
  5. Relay pump                 -> GPIO 16, boleh ditukar

  ThingsBoard:
  - Telemetry HTTP POST setiap 15 minit default
  - Jika shared attribute param_timer_tb wujud, interval ikut nilai tersebut, unit minit

  Telegram:
  - Notification bila sensor keluar range
  - Notification bila semua sensor kembali normal
  - Menu status, parameter, sync ThingsBoard, pump ON/OFF, auto pump
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include "DFRobot_PH.h"

// ===================== USER CONFIG =====================

// WiFi
const char* WIFI_SSID = "SAYUSWA_2.4G";
const char* WIFI_PASS = "no-covid20";

// ThingsBoard
// Contoh: "http://demo.thingsboard.io" atau "https://thingsboard.domain.com"
const String TB_SERVER = "http://demo.thingsboard.io";
const String TB_TOKEN  = "MIjbtLI0rTaTxFulP5OL";//a2f46a70-449c-11f1-9681-6110e8f55c0f

// Telegram
const String TELEGRAM_BOT_TOKEN = "8658402266:AAEQrUoFQCrlMzm9AMIPaxPkkkAS-MKFouw";
const String TELEGRAM_CHAT_ID   = "-5017708130";

// Pin mapping
#ifndef A0
  #define A0 1       // Tukar jika board ESP32-S3 anda tidak define A0
#endif

#ifndef A1
  #define A1 2       // Tukar jika board ESP32-S3 anda tidak define A1
#endif

const int PH_PIN            = A0;
const int TDS_PIN           = A1;
const int DS18B20_PIN       = 14;
const int A01_RX_PIN        = 18;   // Sensor TX -> ESP32-S3 GPIO18
const int A01_TX_PIN        = 17;   // ESP32-S3 GPIO17 -> Sensor RX, jika digunakan
const int RELAY_PUMP_PIN    = 16;

// Relay setting
const bool RELAY_ACTIVE_LOW = true;

// Level setting
const bool  LEVEL_IS_DISTANCE = false;
// Jika LEVEL_IS_DISTANCE = false, level dikira sebagai tinggi tangki - jarak sensor ke permukaan air.
const float TANK_HEIGHT_CM = 150.0;

// ADC setting
const float ADC_VREF_MV = 3300.0;
const int   ADC_MAX     = 4095;

// Timing
const unsigned long SENSOR_READ_INTERVAL_MS   = 2000;
const unsigned long ATTRIBUTE_POLL_INTERVAL_MS = 60000;
const unsigned long TELEGRAM_POLL_INTERVAL_MS = 3000;

// ===================== GLOBAL OBJECTS =====================

OneWire oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);
DFRobot_PH phSensor;
HardwareSerial A01Serial(1);
WiFiClientSecure secureClient;

// ===================== DATA STRUCTURES =====================

struct Params {
  float suhuMin  = 20.0;
  float suhuMax  = 35.0;
  float phMin    = 6.5;
  float phMax    = 8.5;
  float tdsMin   = 0.0;
  float tdsMax   = 1000.0;
  float levelMin = 20.0;
  float levelMax = 120.0;

  uint32_t timerTbMin      = 15;  // param_timer_tb, unit minit
  uint32_t timerRestartMin = 0;   // param_timer_restart, unit minit, 0 = off
};

struct Readings {
  float suhuC      = NAN;
  float ph         = NAN;
  float phVoltageMv = NAN;
  float tdsPpm     = NAN;
  float distanceCm = NAN;
  float levelCm    = NAN;
};

Params cfg;
Readings dataNow;

unsigned long lastSensorReadMs = 0;
unsigned long lastAttrPollMs   = 0;
unsigned long lastTelemetryMs  = 0;
unsigned long lastTelegramMs   = 0;
unsigned long lastWifiTryMs    = 0;

long telegramLastUpdateId = 0;

bool pumpState = false;
bool autoPumpMode = true;
bool alarmActive = false;
String lastAlarmSignature = "";

// ===================== BASIC HELPERS =====================

String fmt(float v, int digits = 2) {
  if (isnan(v)) return "N/A";
  return String(v, digits);
}

unsigned long telemetryIntervalMs() {
  uint32_t minutes = cfg.timerTbMin;
  if (minutes < 1) minutes = 1;
  if (minutes > 1440) minutes = 1440;
  return minutes * 60000UL;
}

void setPump(bool on) {
  pumpState = on;
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(RELAY_PUMP_PIN, on ? LOW : HIGH);
  } else {
    digitalWrite(RELAY_PUMP_PIN, on ? HIGH : LOW);
  }
}

bool beginHttp(HTTPClient &http, const String &url) {
  if (url.startsWith("https://")) {
    return http.begin(secureClient, url);
  }
  return http.begin(url);
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
  }
}

void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  if (millis() - lastWifiTryMs > 10000) {
    lastWifiTryMs = millis();
    connectWiFi();
  }
}

// ===================== ANALOG READ =====================

float readMedianVoltageMv(int pin) {
  const int N = 15;
  int raw[N];

  for (int i = 0; i < N; i++) {
    raw[i] = analogRead(pin);
    delay(5);
  }

  for (int i = 0; i < N - 1; i++) {
    for (int j = i + 1; j < N; j++) {
      if (raw[j] < raw[i]) {
        int tmp = raw[i];
        raw[i] = raw[j];
        raw[j] = tmp;
      }
    }
  }

  float medianRaw = raw[N / 2];
  return (medianRaw * ADC_VREF_MV) / ADC_MAX;
}

// ===================== SENSOR READINGS =====================

float readTemperatureC() {
  tempSensor.requestTemperatures();
  float t = tempSensor.getTempCByIndex(0);

  if (t == DEVICE_DISCONNECTED_C || t < -55 || t > 125) {
    return NAN;
  }
  return t;
}

float readPH(float suhuC) {
  float tempForComp = isnan(suhuC) ? 25.0 : suhuC;
  float voltageMv = readMedianVoltageMv(PH_PIN);
  dataNow.phVoltageMv = voltageMv;

  return phSensor.readPH(voltageMv, tempForComp);
}

float readTDS(float suhuC) {
  float tempForComp = isnan(suhuC) ? 25.0 : suhuC;

  float voltage = readMedianVoltageMv(TDS_PIN) / 1000.0;
  float compensationCoefficient = 1.0 + 0.02 * (tempForComp - 25.0);
  float compensationVoltage = voltage / compensationCoefficient;

  float tds = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage
             - 255.86 * compensationVoltage * compensationVoltage
             + 857.39 * compensationVoltage) * 0.5;

  if (tds < 0) tds = 0;
  return tds;
}

float readA01DistanceCm() {
  uint8_t buf[4];
  unsigned long start = millis();

  while (millis() - start < 150) {
    if (A01Serial.available()) {
      int b = A01Serial.read();

      if (b == 0xFF) {
        buf[0] = 0xFF;

        unsigned long waitStart = millis();
        while (A01Serial.available() < 3 && millis() - waitStart < 30) {
          delay(1);
        }

        if (A01Serial.available() >= 3) {
          buf[1] = A01Serial.read();
          buf[2] = A01Serial.read();
          buf[3] = A01Serial.read();

          uint8_t sum = (buf[0] + buf[1] + buf[2]) & 0xFF;

          if (sum == buf[3]) {
            uint16_t distanceMm = ((uint16_t)buf[1] << 8) | buf[2];

            if (distanceMm > 0) {
              return distanceMm / 10.0;
            }
          }
        }
      }
    }
    delay(1);
  }

  return NAN;
}

float computeLevelCm(float distanceCm) {
  if (isnan(distanceCm)) return NAN;

  if (LEVEL_IS_DISTANCE) {
    return distanceCm;
  }

  float level = TANK_HEIGHT_CM - distanceCm;
  if (level < 0) level = 0;
  if (level > TANK_HEIGHT_CM) level = TANK_HEIGHT_CM;
  return level;
}

void readAllSensors() {
  dataNow.suhuC = readTemperatureC();
  dataNow.ph = readPH(dataNow.suhuC);
  dataNow.tdsPpm = readTDS(dataNow.suhuC);
  dataNow.distanceCm = readA01DistanceCm();
  dataNow.levelCm = computeLevelCm(dataNow.distanceCm);
}

// ===================== THINGSBOARD ATTRIBUTES =====================

bool readJsonFloat(JsonObject obj, const char* key, float &target) {
  if (!obj.containsKey(key)) return false;

  JsonVariant v = obj[key];
  if (v.is<const char*>()) {
    target = String(v.as<const char*>()).toFloat();
  } else {
    target = v.as<float>();
  }
  return true;
}

bool readJsonUInt(JsonObject obj, const char* key, uint32_t &target) {
  if (!obj.containsKey(key)) return false;

  long val;
  JsonVariant v = obj[key];

  if (v.is<const char*>()) {
    val = String(v.as<const char*>()).toInt();
  } else {
    val = v.as<long>();
  }

  if (val < 0) val = 0;
  target = (uint32_t)val;
  return true;
}

bool getSharedAttributes(bool notifyTelegram = false) {
  if (WiFi.status() != WL_CONNECTED) return false;

  String keys =
    "param_suhu_min,param_suhu_max,"
    "param_ph_min,param_ph_max,"
    "param_tds_min,param_tds_max,"
    "param_level_min,param_level_max,"
    "param_timer_tb,param_timer_restart";

  String url = TB_SERVER + "/api/v1/" + TB_TOKEN + "/attributes?sharedKeys=" + keys;

  HTTPClient http;
  if (!beginHttp(http, url)) return false;

  int code = http.GET();
  String payload = http.getString();
  http.end();

  if (code != 200) {
    if (notifyTelegram) {
      // Avoid dependency loop. Telegram send function declared later.
    }
    return false;
  }

  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, payload);
  if (err) return false;

  JsonObject shared;
  if (doc["shared"].is<JsonObject>()) {
    shared = doc["shared"].as<JsonObject>();
  } else {
    shared = doc.as<JsonObject>();
  }

  readJsonFloat(shared, "param_suhu_min", cfg.suhuMin);
  readJsonFloat(shared, "param_suhu_max", cfg.suhuMax);
  readJsonFloat(shared, "param_ph_min", cfg.phMin);
  readJsonFloat(shared, "param_ph_max", cfg.phMax);
  readJsonFloat(shared, "param_tds_min", cfg.tdsMin);
  readJsonFloat(shared, "param_tds_max", cfg.tdsMax);
  readJsonFloat(shared, "param_level_min", cfg.levelMin);
  readJsonFloat(shared, "param_level_max", cfg.levelMax);

  readJsonUInt(shared, "param_timer_tb", cfg.timerTbMin);
  readJsonUInt(shared, "param_timer_restart", cfg.timerRestartMin);

  if (cfg.timerTbMin < 1) cfg.timerTbMin = 15;
  if (cfg.timerTbMin > 1440) cfg.timerTbMin = 1440;

  return true;
}

// ===================== THINGSBOARD TELEMETRY =====================

bool sendTelemetryToThingsBoard() {
  if (WiFi.status() != WL_CONNECTED) return false;

  DynamicJsonDocument doc(2048);

  if (!isnan(dataNow.suhuC))      doc["suhu"] = dataNow.suhuC;
  if (!isnan(dataNow.ph))         doc["ph"] = dataNow.ph;
  if (!isnan(dataNow.tdsPpm))     doc["tds"] = dataNow.tdsPpm;
  if (!isnan(dataNow.distanceCm)) doc["distance_cm"] = dataNow.distanceCm;
  if (!isnan(dataNow.levelCm))    doc["level_cm"] = dataNow.levelCm;

  doc["pump"] = pumpState ? 1 : 0;
  doc["auto_pump"] = autoPumpMode ? 1 : 0;
  doc["wifi_rssi"] = WiFi.RSSI();

  doc["param_suhu_min"] = cfg.suhuMin;
  doc["param_suhu_max"] = cfg.suhuMax;
  doc["param_ph_min"] = cfg.phMin;
  doc["param_ph_max"] = cfg.phMax;
  doc["param_tds_min"] = cfg.tdsMin;
  doc["param_tds_max"] = cfg.tdsMax;
  doc["param_level_min"] = cfg.levelMin;
  doc["param_level_max"] = cfg.levelMax;
  doc["param_timer_tb"] = cfg.timerTbMin;
  doc["param_timer_restart"] = cfg.timerRestartMin;

  doc["sensor_error"] =
    isnan(dataNow.suhuC) ||
    isnan(dataNow.ph) ||
    isnan(dataNow.tdsPpm) ||
    isnan(dataNow.levelCm);

  String payload;
  serializeJson(doc, payload);

  String url = TB_SERVER + "/api/v1/" + TB_TOKEN + "/telemetry";

  HTTPClient http;
  if (!beginHttp(http, url)) return false;

  http.addHeader("Content-Type", "application/json");
  int code = http.POST(payload);
  http.end();

  return code >= 200 && code < 300;
}

// ===================== TELEGRAM =====================

void sendTelegramMessage(const String &text, bool showMenu = false) {
  if (WiFi.status() != WL_CONNECTED) return;
  if (TELEGRAM_BOT_TOKEN.length() < 10 || TELEGRAM_CHAT_ID.length() < 1) return;

  String url = "https://api.telegram.org/bot" + TELEGRAM_BOT_TOKEN + "/sendMessage";

  DynamicJsonDocument doc(4096);
  doc["chat_id"] = TELEGRAM_CHAT_ID;
  doc["text"] = text;

  if (showMenu) {
    JsonObject rm = doc.createNestedObject("reply_markup");
    JsonArray keyboard = rm.createNestedArray("keyboard");

    JsonArray row1 = keyboard.createNestedArray();
    row1.createNestedObject()["text"] = "Status";
    row1.createNestedObject()["text"] = "Parameter";

    JsonArray row2 = keyboard.createNestedArray();
    row2.createNestedObject()["text"] = "Sync TB";
    row2.createNestedObject()["text"] = "Timer";

    JsonArray row3 = keyboard.createNestedArray();
    row3.createNestedObject()["text"] = "Pompa ON";
    row3.createNestedObject()["text"] = "Pompa OFF";
    row3.createNestedObject()["text"] = "Auto Pompa";

    rm["resize_keyboard"] = true;
    rm["one_time_keyboard"] = false;
  }

  String payload;
  serializeJson(doc, payload);

  HTTPClient http;
  http.begin(secureClient, url);
  http.addHeader("Content-Type", "application/json");
  http.POST(payload);
  http.end();
}

String buildStatusMessage() {
  String msg;
  msg += "STATUS SENSOR\n";
  msg += "Suhu: " + fmt(dataNow.suhuC, 2) + " C\n";
  msg += "pH: " + fmt(dataNow.ph, 2) + "\n";
  msg += "TDS: " + fmt(dataNow.tdsPpm, 0) + " ppm\n";
  msg += "Distance: " + fmt(dataNow.distanceCm, 1) + " cm\n";
  msg += "Level: " + fmt(dataNow.levelCm, 1) + " cm\n";
  msg += "Pompa: " + String(pumpState ? "ON" : "OFF") + "\n";
  msg += "Mode pompa: " + String(autoPumpMode ? "AUTO" : "MANUAL") + "\n";
  msg += "WiFi RSSI: " + String(WiFi.RSSI()) + " dBm\n";
  return msg;
}

String buildParamMessage() {
  String msg;
  msg += "PARAMETER THINGSBOARD\n";
  msg += "Suhu min/max: " + fmt(cfg.suhuMin, 2) + " / " + fmt(cfg.suhuMax, 2) + " C\n";
  msg += "pH min/max: " + fmt(cfg.phMin, 2) + " / " + fmt(cfg.phMax, 2) + "\n";
  msg += "TDS min/max: " + fmt(cfg.tdsMin, 0) + " / " + fmt(cfg.tdsMax, 0) + " ppm\n";
  msg += "Level min/max: " + fmt(cfg.levelMin, 1) + " / " + fmt(cfg.levelMax, 1) + " cm\n";
  msg += "Timer TB: " + String(cfg.timerTbMin) + " minit\n";
  msg += "Timer restart: " + String(cfg.timerRestartMin) + " minit, 0 = off\n";
  return msg;
}

void handleTelegramCommand(const String &text, const String &chatId) {
  if (chatId != TELEGRAM_CHAT_ID) return;

  String cmd = text;
  cmd.trim();

  if (cmd == "/start" || cmd == "/menu" || cmd == "Menu") {
    sendTelegramMessage(
      "Menu ESP32-S3 Water Monitoring\n"
      "Pilih menu di bawah.",
      true
    );
  }
  else if (cmd == "/status" || cmd == "Status") {
    sendTelegramMessage(buildStatusMessage(), true);
  }
  else if (cmd == "/param" || cmd == "Parameter") {
    sendTelegramMessage(buildParamMessage(), true);
  }
  else if (cmd == "/sync" || cmd == "Sync TB") {
    bool ok = getSharedAttributes(false);
    sendTelegramMessage(ok ? "Shared attributes ThingsBoard berjaya diselaraskan." :
                             "Gagal sync shared attributes ThingsBoard.", true);
  }
  else if (cmd == "/timer" || cmd == "Timer") {
    sendTelegramMessage(
      "Timer telemetry semasa: " + String(cfg.timerTbMin) + " minit\n"
      "Interval sebenar dikawal oleh shared attribute param_timer_tb.",
      true
    );
  }
  else if (cmd == "/pump_on" || cmd == "Pompa ON") {
    autoPumpMode = false;
    setPump(true);
    sendTelegramMessage("Pompa ON. Mode sekarang MANUAL.", true);
  }
  else if (cmd == "/pump_off" || cmd == "Pompa OFF") {
    autoPumpMode = false;
    setPump(false);
    sendTelegramMessage("Pompa OFF. Mode sekarang MANUAL.", true);
  }
  else if (cmd == "/auto" || cmd == "Auto Pompa") {
    autoPumpMode = true;
    sendTelegramMessage("Mode pompa AUTO diaktifkan.", true);
  }
  else if (cmd == "/help") {
    sendTelegramMessage(
      "Command:\n"
      "/status, /param, /sync, /timer, /pump_on, /pump_off, /auto, /menu",
      true
    );
  }
}

void pollTelegram() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (TELEGRAM_BOT_TOKEN.length() < 10 || TELEGRAM_CHAT_ID.length() < 1) return;

  String url = "https://api.telegram.org/bot" + TELEGRAM_BOT_TOKEN +
               "/getUpdates?timeout=0&offset=" + String(telegramLastUpdateId + 1);

  HTTPClient http;
  http.begin(secureClient, url);
  int code = http.GET();
  String payload = http.getString();
  http.end();

  if (code != 200) return;

  DynamicJsonDocument doc(8192);
  DeserializationError err = deserializeJson(doc, payload);
  if (err) return;

  JsonArray results = doc["result"].as<JsonArray>();

  for (JsonObject update : results) {
    long updateId = update["update_id"].as<long>();
    if (updateId > telegramLastUpdateId) telegramLastUpdateId = updateId;

    if (!update["message"].is<JsonObject>()) continue;

    JsonObject msg = update["message"];
    String text = msg["text"] | "";
    String chatId = String((long long)msg["chat"]["id"]);

    if (text.length() > 0) {
      handleTelegramCommand(text, chatId);
    }
  }
}

// ===================== ALARM AND PUMP LOGIC =====================

bool outOfRange(float value, float minVal, float maxVal) {
  if (isnan(value)) return true;
  return value < minVal || value > maxVal;
}

String buildAlarmSignature() {
  String alarm = "";

  if (outOfRange(dataNow.suhuC, cfg.suhuMin, cfg.suhuMax)) {
    alarm += "Suhu=" + fmt(dataNow.suhuC, 2) +
             " C, range " + fmt(cfg.suhuMin, 2) + "-" + fmt(cfg.suhuMax, 2) + "\n";
  }

  if (outOfRange(dataNow.ph, cfg.phMin, cfg.phMax)) {
    alarm += "pH=" + fmt(dataNow.ph, 2) +
             ", range " + fmt(cfg.phMin, 2) + "-" + fmt(cfg.phMax, 2) + "\n";
  }

  if (outOfRange(dataNow.tdsPpm, cfg.tdsMin, cfg.tdsMax)) {
    alarm += "TDS=" + fmt(dataNow.tdsPpm, 0) +
             " ppm, range " + fmt(cfg.tdsMin, 0) + "-" + fmt(cfg.tdsMax, 0) + "\n";
  }

  if (outOfRange(dataNow.levelCm, cfg.levelMin, cfg.levelMax)) {
    alarm += "Level=" + fmt(dataNow.levelCm, 1) +
             " cm, range " + fmt(cfg.levelMin, 1) + "-" + fmt(cfg.levelMax, 1) + "\n";
  }

  return alarm;
}

void handleAlarmNotification() {
  String alarm = buildAlarmSignature();

  if (alarm.length() > 0) {
    if (!alarmActive || alarm != lastAlarmSignature) {
      alarmActive = true;
      lastAlarmSignature = alarm;

      sendTelegramMessage(
        "ALERT: Bacaan sensor di luar parameter.\n\n" +
        alarm +
        "\n" +
        buildStatusMessage(),
        true
      );
    }
  } else {
    if (alarmActive) {
      alarmActive = false;
      lastAlarmSignature = "";

      sendTelegramMessage(
        "NORMAL: Semua bacaan sensor sudah kembali dalam range parameter.\n\n" +
        buildStatusMessage(),
        true
      );
    }
  }
}

void handlePumpAuto() {
  if (!autoPumpMode) return;
  if (isnan(dataNow.levelCm)) return;

  if (dataNow.levelCm < cfg.levelMin) {
    setPump(true);
  }
  else if (dataNow.levelCm > cfg.levelMax) {
    setPump(false);
  }
}

// ===================== RESTART TIMER =====================

void handleRestartTimer() {
  if (cfg.timerRestartMin == 0) return;

  unsigned long restartMs = cfg.timerRestartMin * 60000UL;

  if (millis() > restartMs) {
    sendTelemetryToThingsBoard();
    sendTelegramMessage("ESP32-S3 restart kerana param_timer_restart telah dicapai.", false);
    delay(1000);
    ESP.restart();
  }
}

// ===================== SETUP AND LOOP =====================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(RELAY_PUMP_PIN, OUTPUT);
  setPump(false);

  analogReadResolution(12);
  analogSetPinAttenuation(PH_PIN, ADC_11db);
  analogSetPinAttenuation(TDS_PIN, ADC_11db);

  EEPROM.begin(512);
  phSensor.begin();

  tempSensor.begin();

  A01Serial.begin(9600, SERIAL_8N1, A01_RX_PIN, A01_TX_PIN);

  secureClient.setInsecure(); 
  // Untuk production, lebih baik guna root CA certificate Telegram dan ThingsBoard HTTPS.

  connectWiFi();

  getSharedAttributes(false);

  readAllSensors();
  lastTelemetryMs = millis() - telemetryIntervalMs();

  sendTelegramMessage(
    "ESP32-S3 Water Monitoring online.\n\n" +
    buildStatusMessage(),
    true
  );
}

void loop() {
  ensureWiFi();

  unsigned long now = millis();

  if (now - lastSensorReadMs >= SENSOR_READ_INTERVAL_MS) {
    lastSensorReadMs = now;

    readAllSensors();

    handlePumpAuto();
    handleAlarmNotification();

    Serial.println(buildStatusMessage());
  }

  if (now - lastAttrPollMs >= ATTRIBUTE_POLL_INTERVAL_MS) {
    lastAttrPollMs = now;
    getSharedAttributes(false);
  }

  if (now - lastTelemetryMs >= telemetryIntervalMs()) {
    lastTelemetryMs = now;
    sendTelemetryToThingsBoard();
  }

  if (now - lastTelegramMs >= TELEGRAM_POLL_INTERVAL_MS) {
    lastTelegramMs = now;
    pollTelegram();
  }

  handleRestartTimer();

  if (!isnan(dataNow.phVoltageMv)) {
    float tempForComp = isnan(dataNow.suhuC) ? 25.0 : dataNow.suhuC;
    phSensor.calibration(dataNow.phVoltageMv, tempForComp);
  }
}