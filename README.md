# Two-wheel self-balancing robot

A two-wheel robot that balances in place on its own, using an ESP32, an MPU6050 IMU and a PID
loop.

Built at XLR8, IIT Bombay, in my first year (Dec 2022).

## What I did

- Built the robot on an ESP32 with an MPU6050 IMU and geared DC motors. The motors have no
  encoders.
- Wrote a PID loop in C++ on the Arduino framework. It maps the tilt angle from the MPU6050 to the
  motor command.
- Tuned the proportional, integral and derivative gains until the robot could balance in place on
  its own.

## Hardware

- ESP32
- MPU6050 IMU
- geared DC motors, no encoders

## The sketch

`balance_bot/balance_bot.ino` is a rewrite, written in 2026 to document the robot. It is not the
original 2022 sketch. The pin numbers and PID gains are starting values, not the gains the robot
was tuned to. The sketch assumes an H-bridge motor driver with two PWM inputs per motor.

What it does on every pass of the loop (a pass that runs the PID ends with a 5 ms wait):

1. Read the accelerometer and gyro registers from the MPU6050 over I2C.
2. Work out the tilt angle with a complementary filter. The gyro rate is integrated for fast
   changes, and the angle is pulled slowly towards the accelerometer's angle so it does not drift.
3. Run PID on the tilt angle. P drives the wheels under the lean. D, taken straight from the gyro
   rate, damps the swing. A small I removes a constant lean.
4. Send the result to both motors as PWM. Past 45 degrees the robot has fallen, and the motors
   stop.

The motors have no encoders, so the loop only knows the tilt and not how far the robot has rolled.

## Tuning the sketch

Start with only P and raise it until the robot oscillates around upright. Add D until the
oscillation dies out. Add a little I last. If the wheels run away from the fall instead of under
it, swap the motor wires or flip the sign of the output.
