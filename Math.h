#ifndef _MATH_H_
#define _MATH_H_

#include <cmath>
#include <cstdlib>

/// <description>
/// PI
/// </description>
const float PI = 3.1415926f;

/// <description>
/// Infinite value.
/// </description>
const float INFINITE_VALUE = 1.0e30f;

/// <description>
/// Epsilon.
/// </description>
const float EPSILON = 1.0e-6f;

/// <description>
/// Return random number bewteen 0 ~ 1.
/// </description>
inline float Random()
{
    return (float)rand() / (float)RAND_MAX;
}

/// <description>
/// 
/// </description>
template <class T>
inline void MinMax(T& min, T& max)
{
    if (min > max)
    {
        T tmp = min;
        min = max;
        max = tmp;
    }
}

/// <description>
/// 2D Vector
/// </description>
class Vector2
{
    public:
        float x, y;

    public:
        Vector2() {}
        Vector2(const float*);
        Vector2(float x, float y);
        // TODO: why no copy constructor
        
        // casting
        operator float* ();
        operator const float* () const;

        // assignment operators
        // a = (b += c) and (a += b) = c are both acceptable.
        Vector2& operator += (const Vector2&);
        Vector2& operator -= (const Vector2&);
        Vector2& operator *= (float);
        Vector2& operator /= (float);

        // unary operator
        Vector2 operator + () const;
        Vector2 operator - () const;

        // binary operator
        Vector2 operator + (const Vector2&) const;
        Vector2 operator - (const Vector2&) const;
        Vector2 operator * (float) const;
        Vector2 operator / (float) const;
        // fit for the pattern: (float)f * (Vector2&)v
        friend Vector2 operator * (float, const Vector2&);

        bool operator == (const Vector2&) const;
        bool operator != (const Vector2&) const;
};

// constructor
inline Vector2::Vector2(const float* pf)
{
    x = pf[0];
    y = pf[1];
}

inline Vector2::Vector2(float fx, float fy)
{
    x = fx;
    y = fy;
}

// casting
inline Vector2::operator float* ()
{
    return (float*)(&x);
}

inline Vector2::operator const float* () const
{
    return (const float*)(&x);
}

// assignment operator
inline Vector2& Vector2::operator += (const Vector2& v)
{
    x += v.x;
    y += v.y;
    return *this;
}

inline Vector2& Vector2::operator -= (const Vector2& v)
{
    x -= v.x;
    y -= v.y;
    return *this;
}

inline Vector2& Vector2::operator *= (float f)
{
    x *= f;
    y *= f;
    return *this;
}

inline Vector2& Vector2::operator /= (float f)
{
    float fInv = 1.0f / f;
    x *= fInv;
    y *= fInv;
    return *this;
}

// unary operators
inline Vector2 Vector2::operator + () const
{   
    return *this;
}

inline Vector2 Vector2::operator - () const
{   
    return Vector2(-x, -y);
}

// binary operators
inline Vector2 Vector2::operator + (const Vector2& v) const
{
    return Vector2(x + v.x, y + v.y);
}

inline Vector2 Vector2::operator - (const Vector2& v) const
{
    return Vector2(x - v.x, y - v.y);
}

inline Vector2 Vector2::operator * (float f) const
{
    return Vector2(x * f, y * f);
}

inline Vector2 Vector2::operator / (float f) const
{
    float fInv = 1.0f / f;
    return Vector2(x * fInv, y * fInv);
}

inline Vector2 operator * (float f, const Vector2& v)
{
    return Vector2(f * v.x, f* v.y);
}

inline bool Vector2::operator == (const Vector2& v) const
{
    return x == v.x && y == v.y;
}

inline bool Vector2::operator != (const Vector2& v) const
{
    return x != v.x || y != v.y;
}

inline float Vec2Dot(const Vector2& v1, const Vector2& v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

inline float Vec2Dot(const Vector2& v)
{
    return v.x * v.x + v.y * v.y;
}

inline float Vec2Perp(const Vector2& v1, const Vector2& v2)
{
    return v1.x * v2.y - v1.y * v2.x;
}

inline float Vec2LengthSquare(const Vector2& v)
{
    return v.x * v.x + v.y * v.y;
}

#endif // Once.