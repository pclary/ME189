#ifndef BLDCMOTOR_H
#define BLDCMOTOR_H


class BldcMotor
{
public:
    BldcMotor(int pin1, int pin2, int pin3, int enablePin = -1);
    BldcMotor(int pin1, int pin2, int pin3);

    void setPosition(float mechDegrees);
    void setPositionOffset(float mechOffset);
    void setCurrentLimit(float currentLimit);
    float getPosition() const;
    void enable();
    void disable();
    bool getEnabled();

    BldcMotor& operator=(float mechDegrees);
    BldcMotor& operator+=(float mechDegrees);
    BldcMotor& operator-=(float mechDegrees);
    operator float() const;
    
private:
    int pin1, pin2, pin3, enablePin;

    float mechDegrees = 0.f;
    float mechOffset = 0.f;
    float currentLimit = 1.f;
    bool enabled = true;

    void configureOutputs();
};

#endif // BLDCMOTOR_H
