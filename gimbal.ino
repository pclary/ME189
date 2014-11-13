#include <Arduino.h>
#include "BldcMotor.h"
#include "utilities.h"
#include "imu.h"
#include <cmath>
#include <Wire.h>


BldcMotor yaw(3, 4, 5);
BldcMotor pitch(20, 9, 10);
BldcMotor roll(23, 22, 21);
int enablePin = 2;


void setup()
{
    Serial.begin(115200);

    pinMode(enablePin, OUTPUT);
    pinMode(13, OUTPUT);

    led_on();
    
    const float cl = 0.3f;
    yaw.setCurrentLimit(cl);
    pitch.setCurrentLimit(cl);
    roll.setCurrentLimit(cl);
    //digitalWrite(enablePin, HIGH);

    imu_init();

    led_off();
}


void loop()
{
    const float qdiv = 1073741824.f;
    float qw = quat[0] / qdiv;
    float qx = quat[1] / qdiv;
    float qy = quat[2] / qdiv;
    float qz = quat[3] / qdiv;

    float roll = std::atan2(2.f*(qw*qx + qy*qz), 1.f - 2.f*(qx*qx + qy*qy));
    float pitch = std::asin(2.f*(qw*qy - qz*qx));
    float yaw = std::atan2(2.f*(qw*qz + qx*qy), 1 - 2.f*(qy*qy + qz*qz));
    
    char buf[256];
    //snprintf(buf, 256, "$%d %d %d %d\n", quat[0], quat[1], quat[2], quat[3]);
    snprintf(buf, 256, "yaw: %7.2f pitch: %7.2f roll: %7.2f\n", yaw*180.f/pi, pitch*180.f/pi, roll*180.f/pi);
    Serial.write(buf);
    
    delay(100);
}
