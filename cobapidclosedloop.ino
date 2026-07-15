#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <RBDdimmer.h>

// === KONFIGURASI PIN ===
#define ZERO_CROSS_PIN 25
#define DIMMER_PIN     26

dimmerLamp dimmer(DIMMER_PIN, ZERO_CROSS_PIN);
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// === PARAMETER PID (dari Ziegler–Nichols Open Loop) ===
double Kp = ;
double Ki = ;
double Kd = ;

double setpoint = 30.0;    // Target suhu °C
double input, output;
double err, lastErr = 0, integral = 0;

unsigned long lastTime = 0;
double sampleTime = 1.0;   // sample time dalam detik

// === SETUP ===
void setup() {
  Serial.begin(115200);
  if (!sht31.begin(0x44)) {
    Serial.println("Sensor SHT31 tidak terdeteksi!");
    while (1) delay(1);
  }

  dimmer.begin(NORMAL_MODE, ON);
  dimmer.setPower(0);  
  Serial.println("time,suhu,setpoint");  // Header CSV untuk MATLAB
}

// === LOOP ===
void loop() {
  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0;

  if (dt >= sampleTime) {
    lastTime = now;

    // Baca sensor suhu
    input = sht31.readTemperature();

    // Hitung error
    err = setpoint - input;

    // Integral
    integral += err * dt;

    // Derivatif
    double derivative = (err - lastErr) / dt;

    // Rumus PID
    output = (Kp * err) + (Ki * integral) + (Kd * derivative);

    // Batasi output (0–100 %)
    if (output > 100) output = 100;
    if (output < 0)   output = 0;

    // Kirim ke dimmer (kontrol heater)
    dimmer.setPower(output);

    // Kirim data ke MATLAB (format: time, suhu, setpoint)
    Serial.print(now / 1000.0, 2); Serial.print(",");
    Serial.print(input, 2);        Serial.print(",");
    Serial.println(setpoint, 2);

    lastErr = err;
  }
}
