#include <Arduino.h>
#include "BldcMotor.h"
#include "utilities.h"
#include <cmath>
#include "inv_mpu_dmp_motion_driver.h"


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

    dmp_load_motion_driver_firmware();
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT);
    dmp_set_fifo_rate(200);
    dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
}


void loop()
{
    float t = millis() / 1000.f;
    const float freq = 5.f;
    
    yaw = std::sin(t*freq) * 60.f;
    pitch = std::sin(t*freq + pi/2.f) * 45.f;
    roll = std::sin(t*freq + pi/4.f) * 180.f;
    
    delay(1);
}
