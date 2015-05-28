#ifndef QMATH_H
#define QMATH_H

#include <array>


typedef std::array<float, 4> Quat;
typedef std::array<float, 3> Rot;

Rot quat2xyz(Quat q);
Rot quat2zxy(Quat q, Rot rlast);
Quat zxy2quat(Rot r);
Quat zyx2quat(Rot r);
Quat normalize(Quat q);


inline Quat operator+(Quat q, float f)
{
    return {q[0] + f, q[1] + f, q[2] + f, q[3] + f};
}

inline Quat operator+(float f, Quat q)
{
    return {q[0] + f, q[1] + f, q[2] + f, q[3] + f};
}

inline Quat operator-(Quat q, float f)
{
    return {q[0] - f, q[1] - f, q[2] - f, q[3] - f};
}

inline Quat operator*(Quat q, float f)
{
    return {q[0] * f, q[1] * f, q[2] * f, q[3] * f};
}

inline Quat operator*(float f, Quat q)
{
    return {q[0] * f, q[1] * f, q[2] * f, q[3] * f};
}

inline Quat operator/(Quat q, float f)
{
    return {q[0] / f, q[1] / f, q[2] / f, q[3] / f};
}

inline Quat& operator+=(Quat& q, float f)
{
    q[0] += f;
    q[1] += f;
    q[2] += f;
    q[3] += f;
    return q;
}

inline Quat& operator-=(Quat& q, float f)
{
    q[0] -= f;
    q[1] -= f;
    q[2] -= f;
    q[3] -= f;
    return q;
}

inline Quat& operator*=(Quat& q, float f)
{
    q[0] *= f;
    q[1] *= f;
    q[2] *= f;
    q[3] *= f;
    return q;
}

inline Quat& operator/=(Quat& q, float f)
{
    q[0] /= f;
    q[1] /= f;
    q[2] /= f;
    q[3] /= f;
    return q;
}

inline Quat operator+(Quat a, Quat b)
{
    return {a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
            a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
            a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
            a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]};
}

inline Quat& operator+=(Quat& a, Quat b)
{
    a[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    a[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    a[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    a[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
    return a;
}

inline Quat operator-(Quat a, Quat b)
{
    b[1] = -b[1];
    b[2] = -b[2];
    b[3] = -b[3];
    return a + b;
}

inline Quat& operator-=(Quat& a, Quat b)
{
    b[1] = -b[1];
    b[2] = -b[2];
    b[3] = -b[3];
    return a += b;
}


inline Rot operator+(Rot r, float f)
{
    return {r[0] + f, r[1] + f, r[2] + f};
}

inline Rot operator+(float f, Rot r)
{
    return {r[0] + f, r[1] + f, r[2] + f};
}

inline Rot operator-(Rot r, float f)
{
    return {r[0] - f, r[1] - f, r[2] - f};
}

inline Rot operator*(Rot r, float f)
{
    return {r[0] * f, r[1] * f, r[2] * f};
}

inline Rot operator*(float f, Rot r)
{
    return {r[0] * f, r[1] * f, r[2] * f};
}

inline Rot operator/(Rot r, float f)
{
    return {r[0] / f, r[1] / f, r[2] / f};
}

inline Rot& operator+=(Rot& r, float f)
{
    r[0] += f;
    r[1] += f;
    r[2] += f;
    return r;
}

inline Rot& operator-=(Rot& r, float f)
{
    r[0] -= f;
    r[1] -= f;
    r[2] -= f;
    return r;
}

inline Rot& operator*=(Rot& r, float f)
{
    r[0] *= f;
    r[1] *= f;
    r[2] *= f;
    return r;
}

inline Rot& operator/=(Rot& r, float f)
{
    r[0] /= f;
    r[1] /= f;
    r[2] /= f;
    return r;
}

#endif