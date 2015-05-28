#include "qmath.h"
#include <cmath>


Rot threeaxisrot(float r11, float r12, float r21, float r31, float r32)
{
    // find angles for rotations about X, Y, and Z axes 
    const float r1 = std::atan2(r11, r12); 
    const float r2 = std::asin(r21); 
    const float r3 = std::atan2(r31, r32);
    return {r1, r2, r3};
}

Rot quat2xyz(Quat q)
{
    return threeaxisrot(-2.f*(q[2]*q[3] - q[0]*q[1]), 
                         q[0]* - q[1]*q[1] - q[2]*q[2] + q[3]*q[3], 
                         2.f*(q[1]*q[3] + q[0]*q[2]), 
                        -2.f*(q[1]*q[2] - q[0]*q[3]), 
                         q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
}

Rot quat2zxy(Quat q, Rot rlast)
{
    const float r31 = 2.f*(q[1]*q[3] - q[0]*q[2]);
    const float r12 = 2.f*(q[1]*q[2] - q[0]*q[3]);
    const float r22 = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
    const float r32 = 2.f*(q[0]*q[1] + q[2]*q[3]);
    const float r33 = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    
    Rot r;
    r[2] = std::atan(-r31/r33);
    r[1] = std::atan2(r32, r33/cos(r[2]));
    
    if (cos(r[1]) > 0)
        r[0] = std::atan2(-r12, r22);
    else
        r[0] = std::atan2(r12, -r22);
    
    // Gimbal lock
    if (r12*r12 + r22*r22 < 1e-6f)
        r = rlast;

    return r;
}


Quat zxy2quat(Rot r)
{
    const float c1 = std::cos(r[0]*0.5f);
    const float c2 = std::cos(r[1]*0.5f);
    const float c3 = std::cos(r[2]*0.5f);
    const float s1 = std::sin(r[0]*0.5f);
    const float s2 = std::sin(r[1]*0.5f);
    const float s3 = std::sin(r[2]*0.5f);
    return {c1*c2*c3 - s1*s2*s3,
            c1*s2*c3 - s1*c2*s3,
            c1*c2*s3 + s1*s2*c3,
            c1*s2*s3 + s1*c2*c3};
}

Quat zyx2quat(Rot r)
{
    const float c1 = std::cos(r[0]*0.5f);
    const float c2 = std::cos(r[1]*0.5f);
    const float c3 = std::cos(r[2]*0.5f);
    const float s1 = std::sin(r[0]*0.5f);
    const float s2 = std::sin(r[1]*0.5f);
    const float s3 = std::sin(r[2]*0.5f);
    return {c1*c2*c3 + s1*s2*s3,
            c1*c2*s3 - s1*s2*c3,
            c1*s2*c3 + s1*c2*s3,
            s1*c2*c3 - c1*s2*s3};
}

Quat normalize(Quat q)
{
    const float norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    return q / norm;
}
