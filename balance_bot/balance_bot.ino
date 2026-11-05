// Two-wheel self-balancing robot: ESP32 + MPU6050 + two geared DC motors.
// Rewritten in 2026 to document the robot. Not the original 2022 sketch.
// Pins and gains are starting values.

#include <Wire.h>

const int MPU = 0x68;  // MPU6050 I2C address

// H-bridge inputs, two PWM pins per motor
const int L_FWD = 25, L_REV = 26;
const int R_FWD = 27, R_REV = 14;

float Kp = 40.0, Ki = 0.5, Kd = 1.2;
float setpoint = 0.0;  // tilt (deg) at which the robot balances

float angle = 0.0;     // filtered tilt (deg)
float integral = 0.0;
unsigned long lastMicros;

void setup() {
  Serial.begin(115200);
  Wire.begin();  // SDA 21, SCL 22 on most ESP32 boards

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // power management register
  Wire.write(0);     // wake the sensor up
  Wire.endTransmission();

  pinMode(L_FWD, OUTPUT);
  pinMode(L_REV, OUTPUT);
  pinMode(R_FWD, OUTPUT);
  pinMode(R_REV, OUTPUT);
  lastMicros = micros();
}

// accel X and Z in g, gyro Y in deg/s
void readImu(float &ax, float &az, float &gy) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // ACCEL_XOUT_H, the next 14 bytes are accel, temp, gyro
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14);

  int16_t raw[7];
  for (int i = 0; i < 7; i++) {
    int hi = Wire.read();
    int lo = Wire.read();
    raw[i] = (hi << 8) | lo;
  }
  ax = raw[0] / 16384.0;  // +-2 g range
  az = raw[2] / 16384.0;
  gy = raw[5] / 131.0;    // +-250 deg/s range
}

void setMotor(int fwdPin, int revPin, int speed) {
  if (speed >= 0) {
    analogWrite(fwdPin, speed);
    analogWrite(revPin, 0);
  } else {
    analogWrite(fwdPin, 0);
    analogWrite(revPin, -speed);
  }
}

void loop() {
  float ax, az, gy;
  readImu(ax, az, gy);

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6;
  lastMicros = now;

  // complementary filter (flip signs if the board is mounted the other way)
  float accAngle = atan2(-ax, az) * 180.0 / PI;
  angle = 0.98 * (angle + gy * dt) + 0.02 * accAngle;

  if (fabs(angle) > 45) {  // fallen over
    setMotor(L_FWD, L_REV, 0);
    setMotor(R_FWD, R_REV, 0);
    integral = 0;
    return;
  }

  float error = angle - setpoint;
  integral = constrain(integral + error * dt, -50, 50);
  float output = Kp * error + Ki * integral + Kd * gy;

  int pwm = constrain((int)output, -255, 255);
  setMotor(L_FWD, L_REV, pwm);
  setMotor(R_FWD, R_REV, pwm);

  delay(5);
}
