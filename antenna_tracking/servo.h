// servo.h
#ifndef SERVO_H
#define SERVO_H

#include <Servo.h>

// Servo constants
const int elevationPin = A1;
const int azimuthPin = A0;
const int servoOffset = 0;
const float SERVO_SPEED_TIME = 0.15;  // seconds per 60 degrees
const float SERVO_DEGREE_INTERVAL = 60.0;

// Create servo object
Servo azimuth;
Servo elevation;

void initializeServo() {
  azimuth.attach(azimuthPin);
  elevation.attach(elevationPin);
}

#endif  // SERVO_H
