#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <RBDdimmer.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>

// === PIN KONFIGURASI ===
#define ZERO_CROSS_PIN 25
#define DIMMER_PIN     26
#define RELAY_FAN       14
#define RELAY_MISTMAKER 27
#define RELAY_EXTRA     13

// === OBJEK SENSOR, DIMMER, LCD ===
Adafruit_SHT31 sht31 = Adafruit_SHT31();
dimmerLamp dimmer(DIMMER_PIN, ZERO_CROSS_PIN);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// === PID PARAMETER ===
double Kp = 10.169, Ki = 0.093, Kd = 203.020;
double setpoint = 30.0, input, output;
double err, lastErr = 0, integral = 0;
unsigned long lastTime = 0;
double sampleTime = 1.0;

// === BATASAN ===
const float TEMP_NORMAL = 30.0;
const float TEMP_HIGH = 30.80;
const float TEMP_LOW  = 29.50;
const float TEMP_TOO_LOW = 29.00;

const float HUM_MM_ON = 80.0;
const float HUM_MIN = 75.0;
const float HUM_MAX = 95.0;

// === WIFI & THINGSPEAK ===
const char* ssid = "RAPI";
const char* password = "Rapi123456";
String apiKey = "AEAMA6BKMO58NAGC";
String talkbackID = "54660";
String talkbackAPIKey = "HL2GC71HUMNIH6LR";
const char* server = "http://api.thingspeak.com";

// === TELEGRAM ===
#define BOTtoken "8274546093:AAF0N0uB1D1IGf8XOV1mJjGU910HdQkPOR0"
#define CHAT_ID "6664413727"
WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

// === STATUS RELAY ===
bool relay1_state = false;
bool relay2_state = false;
bool relay3_state = false;
bool modeManual = false;

// === STATUS NOTIF ===
bool suhuNotifiedHigh = false;
bool suhuNotifiedLow = false;
bool kelembabanNotified = false;

// === TIMER ===
unsigned long lastPeriodicNotif = 0;
const unsigned long notifInterval = 300000;

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // === LCD SEDERHANA ===
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Memulai Sistem");
  lcd.setCursor(0,1);
  lcd.print("Kandang Jangkrik");
  delay(2000);
  lcd.clear();

  // === WIFI ===
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi Connected");

  client.setInsecure();

  // === SENSOR ===
  if (!sht31.begin(0x44)) {
    Serial.println("SHT31 Error");
    while (1);
  }

  // === DIMMER & RELAY ===
  dimmer.begin(NORMAL_MODE, ON);
  dimmer.setPower(0);

  pinMode(RELAY_FAN, OUTPUT);
  pinMode(RELAY_MISTMAKER, OUTPUT);
  pinMode(RELAY_EXTRA, OUTPUT);

  digitalWrite(RELAY_FAN, LOW);
  digitalWrite(RELAY_MISTMAKER, LOW);
  digitalWrite(RELAY_EXTRA, LOW);

  bot.sendMessage(CHAT_ID,
    "✅ ESP32 Kandang Jangkrik aktif dan terhubung ke Telegram!",
    ""
  );

  Serial.println("time,suhu,kelembaban,setpoint,output");
}

// ================== LOOP ==================
void loop() {
  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0;

  if (dt >= sampleTime) {
    lastTime = now;

    input = sht31.readTemperature();
    double humidity = sht31.readHumidity();

    // === PID OTOMATIS (TIDAK DIUBAH) ===
    if (!modeManual) {
      err = setpoint - input;
      integral += err * dt;
      double derivative = (err - lastErr) / dt;
      output = Kp * err + Ki * integral + Kd * derivative;
      output = constrain(output, 0, 100);
      dimmer.setPower(output);
      lastErr = err;

      relay2_state = (humidity < HUM_MM_ON);
      relay1_state = true;

      if (relay2_state) relay3_state = true;
      else {
        if (input >= 31.00) relay3_state = true;
        else if (input <= 30.00) relay3_state = false;
      }
    }

    // === RELAY ===
    digitalWrite(RELAY_FAN, relay1_state ? HIGH : LOW);
    digitalWrite(RELAY_MISTMAKER, relay2_state ? HIGH : LOW);
    digitalWrite(RELAY_EXTRA, relay3_state ? HIGH : LOW);

    // === LCD RUNNING (SEDERHANA) ===
    lcd.setCursor(0,0);
    lcd.print("T:");
    lcd.print(input,1);
    lcd.print((char)223);
    lcd.print("C   ");

    lcd.setCursor(0,1);
    lcd.print("H:");
    lcd.print(humidity,1);
    lcd.print("%   ");

    Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f\n",
                  now/1000, input, humidity, setpoint, output);

    checkConditions(input, humidity);

    if (now - lastPeriodicNotif > notifInterval) {
      lastPeriodicNotif = now;
      sendPeriodicUpdate(input, humidity);
    }

    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      String url = String(server) + "/update?api_key=" + apiKey +
                   "&field1=" + String(input, 2) +
                   "&field2=" + String(humidity, 2) +
                   "&field3=" + String(relay1_state) +
                   "&field4=" + String(relay2_state) +
                   "&field5=" + String(relay3_state);
      http.begin(url);
      http.GET();
      http.end();
    }

    checkTalkBack();
    delay(15000);
  }
}

// ================== TELEGRAM (LENGKAP) ==================
void checkConditions(float suhu, float kelembaban) {

  if (suhu > TEMP_HIGH && !suhuNotifiedHigh) {
    bot.sendMessage(CHAT_ID,
      "🔥 *Peringatan!* Suhu terlalu tinggi!\n📊 Suhu: " + String(suhu,2) + "°C",
      "Markdown");
    suhuNotifiedHigh = true;
    suhuNotifiedLow = false;
  }
  else if (suhu < TEMP_LOW && !suhuNotifiedLow) {
    if (suhu < TEMP_TOO_LOW) {
      bot.sendMessage(CHAT_ID,
        "⚠️ *Bahaya!* Suhu terlalu rendah!\n📊 Suhu: " + String(suhu,2) + "°C",
        "Markdown");
    } else {
      bot.sendMessage(CHAT_ID,
        "❄️ *Peringatan!* Suhu di bawah normal\n📊 Suhu: " + String(suhu,2) + "°C",
        "Markdown");
    }
    suhuNotifiedLow = true;
    suhuNotifiedHigh = false;
  }
  else if (suhu >= TEMP_LOW && suhu <= TEMP_HIGH) {
    suhuNotifiedLow = false;
    suhuNotifiedHigh = false;
  }

  if ((kelembaban < HUM_MIN || kelembaban > HUM_MAX) && !kelembabanNotified) {
    bot.sendMessage(CHAT_ID,
      "💧 *Peringatan!* Kelembaban tidak normal\n📊 RH: " + String(kelembaban,2) + "%",
      "Markdown");
    kelembabanNotified = true;
  }
  else if ((kelembaban >= HUM_MIN && kelembaban <= HUM_MAX) && kelembabanNotified) {
    bot.sendMessage(CHAT_ID,
      "✅ Kelembaban kembali normal (" + String(kelembaban,2) + "%)",
      "Markdown");
    kelembabanNotified = false;
  }
}

void sendPeriodicUpdate(float suhu, float kelembaban) {
  String msg = "📡 *Update Berkala Kandang Jangkrik*\n";
  msg += "🌡 Suhu: " + String(suhu,2) + "°C\n";
  msg += "💧 Kelembaban: " + String(kelembaban,2) + "%";
  bot.sendMessage(CHAT_ID, msg, "Markdown");
}

void checkTalkBack() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(server) + "/talkbacks/" + talkbackID +
                 "/commands/execute?api_key=" + talkbackAPIKey;
    http.begin(url);

    if (http.GET() == 200) {
      String cmd = http.getString();
      cmd.trim();

      if (cmd == "RELAY1_ON") { relay1_state = true; modeManual = true; }
      else if (cmd == "RELAY1_OFF") { relay1_state = false; modeManual = true; }
      else if (cmd == "RELAY2_ON") { relay2_state = true; modeManual = true; }
      else if (cmd == "RELAY2_OFF") { relay2_state = false; modeManual = true; }
      else if (cmd == "RELAY3_ON") { relay3_state = true; modeManual = true; }
      else if (cmd == "RELAY3_OFF") { relay3_state = false; modeManual = true; }
      else if (cmd == "AUTO") { modeManual = false; }

      if (modeManual) dimmer.setPower(0);
    }
    http.end();
  }
}
