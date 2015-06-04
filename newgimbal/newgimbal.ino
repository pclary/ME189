#include <Arduino.h>
#include "BldcMotor.h"
#include "utilities.h"
#include "imu.h"
#include <cmath>
#include <SPI.h>
#include "ControlLoop.h"
#include "qmath.h"
#include "LowPass.h"


const int potAPin = A10;
const int potBPin = A11;
const int potCPin = A14;
const float pot2rad = pi * 330.f / 4096.f / 180.f;
const float rad2edeg = 180.f * 4.f / pi;
const int potAOffset = 2048;
const int potBOffset = 1950;//2048;
const int potCOffset = 1950;//2048;
float potATrim = 0.f;
float potBTrim = 0.f;
float potCTrim = 0.f;

LowPass yawLP;

BldcMotor yawMotor(5, 3, 4, 0);
BldcMotor rollMotor(20, 6, 9, 1);
BldcMotor pitchMotor(21, 23, 22, 2);
const float maxCurrent = 0.5f;

const float dt = 1.f/200.f;
ControlLoop yawLoop(dt);
ControlLoop pitchLoop(dt);
ControlLoop rollLoop(dt);

Rot rcom = {0.f, 0.f, 0.f};
Rot rctrl;
Rot rimuraw;

float degmod(float deg);


void setup()
{
    asm(".global _printf_float");
    
    Serial.begin(115200);
    
    analogReadResolution(12);

    yawLP.setCutoffFreq(0.3f, dt);
    
    yawLoop.setOutputLimits(-maxCurrent, maxCurrent);
    rollLoop.setOutputLimits(-maxCurrent, maxCurrent);
    pitchLoop.setOutputLimits(-maxCurrent, maxCurrent);

    yawLoop.setTuning(0.f, 0.f, 0.f);
    rollLoop.setTuning(1.f, 30.f, 0.1f);
    pitchLoop.setTuning(1.f, 100.f, 0.1f);

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

    potATrim = -degmod(rstartup[0]*rad2edeg);
    potBTrim = -degmod(rstartup[1]*rad2edeg);
    potCTrim = -degmod(rstartup[2]*rad2edeg);
    
    rctrl = rstartup;
    
    yawMotor.setOutput(0.f, 0.f);
    rollMotor.setOutput(0.f, 0.f);
    pitchMotor.setOutput(0.f, 0.f);
    
    imu_init();

    Serial.write("test\n");
}


void loop()
{
    char buf[256];
    
    // Wait for a new IMU update
    while (!imu_check_update());

    // Get potentiometer data
    const Rot rpot = {(analogRead(potAPin) - potAOffset)*pot2rad*0.f,
                      (analogRead(potBPin) - potBOffset)*pot2rad,
                      (analogRead(potCPin) - potCOffset)*pot2rad};
    const Quat qpot = zxy2quat(rpot);

    // Get IMU orientation data
    const float qdiv = 1073741824.f;
    const Quat qimuraw = {quat[0] / qdiv, 
                          quat[1] / qdiv, 
                          quat[2] / qdiv, 
                          quat[3] / qdiv};
    rimuraw = quat2zxy(qimuraw, rimuraw);
    yawLP.push(rimuraw[0]);
    const Rot ryawlp = {-yawLP, 0.f, 0.f};
    const Quat qyawlp = zxy2quat(ryawlp);
    const Quat qimu = qyawlp + qimuraw;
    
    // Get commanded orientation
    const Quat qcom = zyx2quat(rcom);
 
    // Get desired potentiometer positions
    const Quat qctrl = (qcom - qimu) + qpot;
    rctrl = quat2zxy(qctrl, rctrl);

    // Update PID loops
    const float yawCurrent = yawLoop.update(rctrl[0] - rpot[0]);
    const float rollCurrent = rollLoop.update(rctrl[1] - rpot[1]);
    const float pitchCurrent = pitchLoop.update(rctrl[2] - rpot[2]);

    // Send commands to motors
    yawMotor.setCurrent(0.f);//yawCurrent);
    rollMotor.setCurrent(rollCurrent);
    pitchMotor.setCurrent(pitchCurrent);
    yawMotor.update(rpot[0]*rad2edeg + potATrim);
    rollMotor.update(rpot[1]*rad2edeg + potBTrim);
    pitchMotor.update(rpot[2]*rad2edeg + potCTrim);

    const Rot rimu = quat2zxy(qimu, {0.f, 0.f, 0.f});
    
    
    snprintf(buf, 256, "imuraw: %2.4f, %1.4f, %1.4f\n", rimuraw[0], rimuraw[1], rimuraw[2]);
    Serial.write(buf);
    snprintf(buf, 256, "imu: %2.4f, %1.4f, %1.4f\n", rimu[0], rimu[1], rimu[2]);
    Serial.write(buf);
    // snprintf(buf, 256, "pot:  %2.4f, %1.4f, %1.4f\n", rpot[0], rpot[1], rpot[2]);
    // snprintf(buf, 256, "pot:  %.4d, %.4d, %.4d\n", analogRead(potAPin), analogRead(potBPin), analogRead(potCPin));
    // Serial.write(buf);
    // snprintf(buf, 256, "ctrl: %2.4f, %1.4f, %1.4f\n", rctrl[0], rctrl[1], rctrl[2]);
    // Serial.write(buf);
    
    // snprintf(buf, 256, "I: %2.4f, %1.4f, %1.4f\n", yawCurrent, rollCurrent, pitchCurrent);
    // Serial.write(buf);
}


float degmod(float deg)
{
    const float a = std::fmod(deg + 180.f, 360.f);
    const float b = a > 0.f ? a : a + 360.f;
    return b - 180.f;
}
