#include <Arduino.h>
#include "BldcMotor.h"
#include "utilities.h"
#include "imu.h"
#include <cmath>
#include <Wire.h>
#include "ControlLoop.h"


BldcMotor yawMotor(3, 4, 5);
BldcMotor pitchMotor(20, 9, 10);
BldcMotor rollMotor(23, 22, 21);
int enablePin = 2;

const float dt = 1.f/200.f;
ControlLoop yawLoop(dt);
ControlLoop pitchLoop(dt);
ControlLoop rollLoop(dt);

LowPass yawLP;


void setup()
{
    Serial.begin(115200);

    pinMode(enablePin, OUTPUT);
    pinMode(13, OUTPUT);

    yawLoop.setOutputLimits(-300.f, 300.f);
    pitchLoop.setOutputLimits(-400.f, 400.f);
    rollLoop.setOutputLimits(-800.f, 800.f);

    yawLoop.setKp(10.f);
    pitchLoop.setKp(10.f);
    rollLoop.setKp(40.f);

    yawLP.setCutoffFreq(0.1f, dt);

    delay(500);
    led_on();
    delay(100);
    led_off();
    delay(2000);

    led_on();
    
    const float cl = 0.3;
    yawMotor.setCurrentLimit(cl);
    pitchMotor.setCurrentLimit(cl);
    rollMotor.setCurrentLimit(cl);
    digitalWrite(enablePin, HIGH);

    delay(500);

    imu_init();

    delay(1000);

    led_off();
    
}


void loop()
{
    // Wait for a new IMU update
    while (!imu_check_update());

    // Get quaternion data
    const float qdiv = 1073741824.f;
    float qw = quat[0] / qdiv;
    float qx = quat[1] / qdiv;
    float qy = quat[2] / qdiv;
    float qz = quat[3] / qdiv;

    // Compute euler angles
    float yaw = std::atan2(2.f*(qw*qz + qx*qy), 1 - 2.f*(qy*qy + qz*qz));
    float pitch = std::asin(2.f*(qw*qy - qz*qx));
    float roll = std::atan2(2.f*(qw*qx + qy*qz), 1.f - 2.f*(qx*qx + qy*qy));

    yaw *= 180.f/pi;
    pitch *= 180.f/pi;
    roll *= 180.f/pi;

    yawLP.push(yaw);
    float yawHP = yaw - yawLP;
    
    char buf[256];
    //snprintf(buf, 256, "yaw: %7.2f pitch: %7.2f roll: %7.2f\n", yaw, pitch, roll);
    snprintf(buf, 256, "%f,%f\n", yaw, float(yawLoop.derror));
    Serial.write(buf);
    
    yawMotor += yawLoop.update(deadband(-yawHP, 5.f)) * dt;
    pitchMotor += pitchLoop.update(-pitch) * dt;
    rollMotor += rollLoop.update(-roll) * dt;
}
