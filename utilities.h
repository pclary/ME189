#ifndef UTILITIES_H
#define UTILITIES_H

#include <cmath>
#include "Arduino.h"


const float pi = 3.1415926f;


inline float trapezoid(float phase)
{
    phase = phase - pi/3.f;
    
    phase = std::fmod(phase, 2.f*pi);
    if (phase < 0.f)
        phase += pi*2.f;
    
    if (phase < pi/3.f)
        return 1.f;
    else if (phase < pi)
        return 2.f - phase / (pi/3.f);
    else if (phase < pi*4.f/3.f)
        return -1.f;
    else
        return -5.f + phase / (pi/3.f);
}


inline void led_on()
{
    digitalWrite(13, HIGH);
}


inline void led_off()
{
    digitalWrite(13, LOW);
}

#endif // UTILITIES_H
