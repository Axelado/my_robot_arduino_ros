#include "Motor.h"

Motor::Motor(unsigned directionControlPin1,
             unsigned directionControlPin2,
             unsigned speedControlPin) :
             directionControlPin1(directionControlPin1),
             directionControlPin2(directionControlPin2),
             speedControlPin(speedControlPin) {}

void Motor::init() {
    pinMode(directionControlPin1, OUTPUT);
    pinMode(directionControlPin2, OUTPUT);
    pinMode(speedControlPin, OUTPUT);
}

// init before use the following methods
void Motor::moveForward(unsigned pwmSpeed) {
    digitalWrite(directionControlPin1, LOW);
	digitalWrite(directionControlPin2, HIGH);
    analogWrite(speedControlPin, pwmSpeed);
}

void Motor::moveBackward(unsigned pwmSpeed) {
    digitalWrite(directionControlPin1, HIGH);
	digitalWrite(directionControlPin2, LOW);
    analogWrite(speedControlPin, pwmSpeed);
}

void Motor::stop() {
    digitalWrite(directionControlPin1, LOW);
	digitalWrite(directionControlPin2, LOW);
}       