#ifndef BLDCMOTOR_H
#define BLDCMOTOR_H


class BldcMotor
{
public:
    BldcMotor(int pin1, int pin2, int pin3);

    void setPosition(float mechDegrees);
    float getPosition() const;
    void setPositionOffset(float mechOffset);
    void setCurrentLimit(float currentLimit);

    BldcMotor& operator=(float mechDegrees);
    BldcMotor& operator+=(float mechDegrees);
    BldcMotor& operator-=(float mechDegrees);
    operator float() const;
    
private:
    int pin1, pin2, pin3;

    float mechDegrees = 0.f;
    float mechOffset = 0.f;
    float currentLimit = 1.f;

    void configureOutputs();
};

#endif // BLDCMOTOR_H
