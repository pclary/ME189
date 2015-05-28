#include <Arduino.h>
#include "BldcMotor.h"
#include "utilities.h"
#include "imu.h"
#include <cmath>
#include <SPI.h>
#include "ControlLoop.h"
#include "qmath.h"


const int potAPin = A10;
const int potBPin = A11;
const int potCPin = A14;
const float pot2rad = pi * 330.f / 4096.f / 180.f;
const float rad2edeg = 180.f * 4.f / pi;
const int potAOffset = 2048;
const int potBOffset = 2048;
const int potCOffset = 2048;
float potATrim = 0.f;
float potBTrim = 0.f;
float potCTrim = 0.f;

BldcMotor yawMotor(5, 3, 4, 0);
BldcMotor rollMotor(20, 6, 9, 1);
BldcMotor pitchMotor(21, 23, 22, 2);
const float maxCurrent = 0.3f;

const float dt = 1.f/200.f;
ControlLoop yawLoop(dt);
ControlLoop pitchLoop(dt);
ControlLoop rollLoop(dt);

Rot rcom = {0.f, 0.f, 0.f};
Rot rctrl;

float degmod(float deg);


void setup()
{
    asm(".global _printf_float");
    
    Serial.begin(115200);
    
    delay(2000);
    
    analogReadResolution(12);

    yawLoop.setOutputLimits(-maxCurrent, maxCurrent);
    rollLoop.setOutputLimits(-maxCurrent, maxCurrent);
    pitchLoop.setOutputLimits(-maxCurrent, maxCurrent);

    yawLoop.setTuning(0.f, 0.f, 0.f);
    rollLoop.setTuning(0.f, 0.f, 0.f);
    pitchLoop.setTuning(1.f, 0.f, 0.f);

    yawMotor.enable();
    rollMotor.enable();
    pitchMotor.enable();
    
    yawMotor.setOutput(maxCurrent, 0.f);
    rollMotor.setOutput(maxCurrent, 0.f);
    pitchMotor.setOutput(maxCurrent, 0.f);
    
    delay(1000);
    
    const Rot rstartup = {(analogRead(potAPin) - potAOffset)*pot2rad,
                          (analogRead(potBPin) - potBOffset)*pot2rad,
                          (analogRead(potCPin) - potCOffset)*pot2rad};

    potATrim = -degmod(rstartup[0]*rad2edeg)/rad2edeg;
    potBTrim = -degmod(rstartup[1]*rad2edeg)/rad2edeg;
    potCTrim = -degmod(rstartup[2]*rad2edeg)/rad2edeg;
    
    rctrl = rstartup;
    
    yawMotor.setOutput(0.f, 0.f);
    rollMotor.setOutput(0.f, 0.f);
    pitchMotor.setOutput(0.f, 0.f);
    
    imu_init();
}


void loop()
{
    // Wait for a new IMU update
    while (!imu_check_update());

    // Get potentiometer data
    const Rot rpot = {(analogRead(potAPin) - potAOffset)*pot2rad + potATrim,
                      (analogRead(potBPin) - potBOffset)*pot2rad + potBTrim,
                      (analogRead(potCPin) - potCOffset)*pot2rad + potCTrim};
    const Quat qpot = zxy2quat(rpot);

    // Get IMU orientation data
    const float qdiv = 1073741824.f;
    const Quat qimu = {quat[0] / qdiv, 
                       quat[1] / qdiv, 
                       quat[2] / qdiv, 
                       quat[3] / qdiv};
                       
    // Get commanded orientation
    const Quat qcom = zyx2quat(rcom);
 
    // Get desired potentiometer positions
    const Quat qctrl = (qcom - qimu) + qpot;
    rctrl = quat2zxy(qctrl, rctrl);

    // Update PID loops
    const float yawCurrent = yawLoop.update(rctrl[0]);
    const float rollCurrent = rollLoop.update(rctrl[1]);
    const float pitchCurrent = pitchLoop.update(rctrl[2]);

    // Send commands to motors
    yawMotor.setCurrent(0.3f);//yawCurrent);
    rollMotor.setCurrent(0.3f);//rollCurrent);
    pitchMotor.setCurrent(0.3f);//pitchCurrent);
    yawMotor.update(rpot[0]*rad2edeg);
    rollMotor.update(rpot[1]*rad2edeg);
    pitchMotor.update(rpot[2]*rad2edeg);

    //const Rot rimu = quat2zxy(qimu, {0.f, 0.f, 0.f});
    
    char buf[256];
    //snprintf(buf, 256, "%1.4f, %1.4f, %1.4f\n", rimu[0], rimu[1], rimu[2]);
    snprintf(buf, 256, "%1.4f, %1.4f, %1.4f\n", rpot[0], rpot[1], rpot[2]);
    Serial.write(buf);
}


float degmod(float deg)
{
    const float a = std::fmod(deg + 180.f, 360.f);
    const float b = a > 0.f ? a : a + 360.f;
    return b - 180.f;
}
