#include "BldcMotor.h"
#include "Arduino.h"
#include "utilities.h"


BldcMotor::BldcMotor(int pin1, int pin2, int pin3) : pin1(pin1), pin2(pin2), pin3(pin3)
{
    const int pwmFrequency = 20000;
    
    analogWriteFrequency(pin1, pwmFrequency);
    analogWriteFrequency(pin2, pwmFrequency);
    analogWriteFrequency(pin3, pwmFrequency);

    analogWriteResolution(12);
}


void BldcMotor::setPosition(float md)
{
    mechDegrees = md - mechOffset;
    configureOutputs();
}


void BldcMotor::setPositionOffset(float mo)
{
    mechOffset = mo;
}


void BldcMotor::setCurrentLimit(float cl)
{
    currentLimit = cl;
    configureOutputs();
}


float BldcMotor::getPosition() const
{
    return mechDegrees + mechOffset;
}


BldcMotor& BldcMotor::operator=(float md)
{
    setPosition(md);
    return *this;
}


BldcMotor& BldcMotor::operator+=(float value)
{
    setPosition(getPosition() + value);
    return *this;
}


BldcMotor& BldcMotor::operator-=(float value)
{
    setPosition(getPosition() + value);
    return *this;
}


BldcMotor::operator float() const
{
    return getPosition();
}


void BldcMotor::configureOutputs()
{
    const float mechDegElecRadConv = 7.f * pi / 180.f;
    const int maxPwmInt = 4095;

    float ph1 = mechDegrees * mechDegElecRadConv;
    float ph2 = ph1 + pi*2.f/3.f;
    float ph3 = ph1 + pi*4.f/3.f;

    int pwmval1 = int((std::sin(ph1) + 1.f) / 2.f * maxPwmInt * currentLimit);
    int pwmval2 = int((std::sin(ph2) + 1.f) / 2.f * maxPwmInt * currentLimit);
    int pwmval3 = int((std::sin(ph3) + 1.f) / 2.f * maxPwmInt * currentLimit);
    
    analogWrite(pin1, pwmval1);
    analogWrite(pin2, pwmval2);
    analogWrite(pin3, pwmval3);
}
