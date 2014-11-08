#include <Arduino.h>
#include "BldcMotor.h"
#include <cmath>


BldcMotor yaw(3, 4, 5);
BldcMotor pitch(20, 9, 10);
BldcMotor roll(23, 22, 21);
int enablePin = 2;


void setup()
{
    const float cl = 0.3f;
    yaw.setCurrentLimit(cl);
    pitch.setCurrentLimit(cl);
    roll.setCurrentLimit(cl);

    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH);
}

void loop()
{
    const float pi = 3.1415926f;
    float t = millis() / 1000.f;
    const float freq = 2.f;
    
    yaw = std::sin(t*freq) * 60.f;
    pitch = std::sin(t*freq + pi/3.f) * 45.f;
    roll = std::sin(t*freq + pi/2.f) * 180.f;
    
    delay(10);
}
