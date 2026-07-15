#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <RBDdimmer.h>

#define HEATER_PIN 26
#define ZC_PIN 25

Adafruit_SHT31 sht31 = Adafruit_SHT31();
dimmerLamp dimmer(HEATER_PIN, ZC_PIN);

float powerLevel = 0;
unsigned long startTime;

void setup() {
Serial.begin(115200);
if (!sht31.begin(0x44)) { // alamat default 0x44
Serial.println("SHT31 not found!");
while (1) delay(1);
}
dimmer.begin(NORMAL_MODE, ON);
dimmer.setPower(0);
startTime = millis();
}

void loop() {
// === Baca perintah dari MATLAB jika ada ===
if (Serial.available() > 0) {
String cmd = Serial.readStringUntil('\n');
cmd.trim(); // hapus spasi/CR

// Format perintah: P:30 atau langsung 30
if (cmd.startsWith("P:") || cmd.startsWith("p:")) {
  powerLevel = cmd.substring(2).toFloat();
} else {
  powerLevel = cmd.toFloat();
}

powerLevel = constrain(powerLevel, 0, 100);
}

// === Selalu set daya heater (bertahan di level terakhir) ===
dimmer.setPower(powerLevel);

// === Baca suhu dari SHT31 ===
float suhu = sht31.readTemperature();
if (isnan(suhu)) suhu = 0;

// === Kirim data ke MATLAB: waktu,suhu,daya ===
float waktu = (millis() - startTime) / 1000.0;
Serial.print(waktu, 2);
Serial.print(",");
Serial.print(suhu, 2);
Serial.print(",");
Serial.println(powerLevel, 1);

delay(1000); // kirim setiap 1 detik
}