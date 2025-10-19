#include <Wire.h>

// ICM20602 I2C address
#define ICM20602_ADDR 0x69

// Registers in the sensor
#define PWR_MGMT_1    0x6B
#define GYRO_CONFIG   0x1B
#define GYRO_XOUT_H   0x43

// Sensitivity scale for ±250 dps
#define GYRO_SENS 131.0  

// ----------------- I2C Helper -----------------
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ICM20602_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t read16bit(uint8_t reg) {
  Wire.beginTransmission(ICM20602_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM20602_ADDR, 2, true);
  return (Wire.read() << 8) | Wire.read();
}

// ----------------- Yaw Calculation -----------------
float getYaw() {
  static float yawAngle = 0.0;
  static unsigned long lastTime = micros();

  // Read raw gyro Z
  int16_t gz_raw = read16bit(GYRO_XOUT_H + 4);
  float gz = gz_raw / GYRO_SENS;  // °/s

  // Time delta
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0; // sec
  lastTime = now;

  // Change in yaw
  float deltaYaw = gz * dt;

  // -------- Deadband filter --------
  if (fabs(deltaYaw) > 0.006) {   // ignore very small drift
    yawAngle += deltaYaw;
  }

  return yawAngle;
}

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(18, 19); // ESP32 SDA=21, SCL=22

  // Init ICM20602
  writeRegister(PWR_MGMT_1, 0x00); // Wake up
  delay(100);
  writeRegister(PWR_MGMT_1, 0x01); // Auto select clock
  writeRegister(GYRO_CONFIG, 0x00); // ±250 dps

  Serial.println("ICM20602 Ready.");
}

// ----------------- Loop -----------------
void loop() {
  float yaw = getYaw();
  Serial.println(yaw);
  delay(10);
}
