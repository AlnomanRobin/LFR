#include "Config.h"

void motorInit() {
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);

  pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  motorStop();
}

static void driveChannel(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, int16_t speed, uint8_t invert) {
  if (invert) {
    speed = -speed;
  }

  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, (uint8_t)speed);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, (uint8_t)(-speed));
  } else {
    // Active short brake at zero speed for crisp response
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, 0);
  }
}

void setMotors(int16_t leftSpeed, int16_t rightSpeed) {
  driveChannel(PIN_PWMA, PIN_AIN1, PIN_AIN2, leftSpeed, cfg.invertLeftMotor);
  driveChannel(PIN_PWMB, PIN_BIN1, PIN_BIN2, rightSpeed, cfg.invertRightMotor);
}

void motorBrake() {
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, HIGH);
  analogWrite(PIN_PWMA, 255);

  digitalWrite(PIN_BIN1, HIGH);
  digitalWrite(PIN_BIN2, HIGH);
  analogWrite(PIN_PWMB, 255);
}

void motorStop() {
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, 0);

  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);
  analogWrite(PIN_PWMB, 0);
}

