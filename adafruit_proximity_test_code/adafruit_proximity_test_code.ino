#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define XSHUT_PIN 11

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);

  pinMode(XSHUT_PIN, OUTPUT);

  // ===== HARD RESET SENSOR =====
  digitalWrite(XSHUT_PIN, LOW);
  delay(10);
  digitalWrite(XSHUT_PIN, HIGH);
  delay(10);

  Serial.println("VL53L0X with XSHUT Test");

  // ===== INIT SENSOR =====
  if (!lox.begin(0x29)) {
    Serial.println("❌ Failed to boot VL53L0X");
    while (1);
  }

  Serial.println("✅ Sensor Ready!");
}

void loop() {

  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println("Out of range");
  }

  delay(200);
}