#pragma once

#include <cmath>

struct __attribute__((packed)) MyMatrix {
    // column major layout
    float buf[16] = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f};

    MyMatrix() = default;

    void print();
    float get(unsigned char row, unsigned char col) const;
    void set(unsigned char row, unsigned char col, const float f);
    void reset();
    MyMatrix& rotateX(const double angleInRad);
    MyMatrix& rotateZ(const double angleInRad);
    MyMatrix& rotateY(const double angleInRad);
    MyMatrix operator*(const MyMatrix rhs) const;
    MyMatrix transpose() const;
};

struct MyQuaternion;
struct __attribute__((packed)) MyPoint {
    float x,y,z;

    MyPoint() : x(0.0), y(0.0), z(0.0) {}
    constexpr MyPoint(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}

    MyPoint operator+(const MyPoint &rhs) const;
    MyPoint operator-(const MyPoint &rhs) const;
    MyPoint& operator+=(const MyPoint &rhs);
    MyPoint operator*(float m) const;
    MyPoint& operator*=(float m);
    MyPoint opposite() const;
    void print() const;
    MyPoint transform(const MyMatrix& m) const;
    MyPoint transform(const MyQuaternion& q) const;
    float dot(const MyPoint& rhs) const;
    MyPoint cross(const MyPoint& rhs) const;
    float length() const;
    void normalize();
};

struct MyQuaternion {
    float w = 1.0;
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;

    MyQuaternion() = default;
    MyQuaternion(float _w, float _x, float _y, float _z)
        : w(_w), x(_x), y(_y), z(_z) {}

    void setRotation(float angle, MyPoint axis);
    void rotateX(float angle);
    void rotateY(float angle);
    void rotateZ(float angle);

    float magnitude() const;
    void toOppositeAxis();

    void normalize();

    MyQuaternion operator*(float mult) const;
    MyQuaternion& operator*=(float mult);
    MyQuaternion operator-(const MyQuaternion& rhs) const;
    MyQuaternion operator+(const MyQuaternion& rhs) const;
    MyQuaternion operator*(const MyQuaternion& rhs) const;

    float dot(const MyQuaternion& rhs) const;

    MyMatrix toMatrix() const;

    static MyQuaternion slerp(const MyQuaternion& start, MyQuaternion end,
                              float t);
};

inline
float MyMatrix::get(unsigned char row, unsigned char col) const
{
    return buf[4*col + row];
}

inline
void MyMatrix::set(unsigned char row, unsigned char col, const float f)
{
    buf[4*col + row] = f;
}

inline
MyPoint MyPoint::operator+(const MyPoint &rhs) const
{
    MyPoint ret = *this;
    ret.x += rhs.x;
    ret.y += rhs.y;
    ret.z += rhs.z;

    return ret;
}

inline
MyPoint MyPoint::operator-(const MyPoint &rhs) const
{
    MyPoint ret = *this;
    ret.x -= rhs.x;
    ret.y -= rhs.y;
    ret.z -= rhs.z;

    return ret;
}

inline
MyPoint& MyPoint::operator+=(const MyPoint &rhs)
{
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;

    return *this;
}

inline
MyPoint MyPoint::operator*(float m) const
{
    return MyPoint(x * m, y * m, z * m);
}

inline
MyPoint& MyPoint::operator*=(float m)
{
    x *= m;
    y *= m;
    z *= m;
    return *this;
}

inline
MyPoint MyPoint::opposite() const
{
    return MyPoint(-x, -y, -z);
}

inline
MyPoint MyPoint::cross(const MyPoint& rhs) const
{
    MyPoint ret;
    ret.x = y*rhs.z - z * rhs.y;
    ret.y = z*rhs.x - x * rhs.z;
    ret.z = x*rhs.y - y * rhs.x;
    return ret;
}

inline
float MyPoint::dot(const MyPoint& rhs) const
{
    return x*rhs.x+y*rhs.y+z*rhs.z;
}

inline
float MyPoint::length() const
{
    return sqrtf(x*x+y*y+z*z);
}

inline
void MyPoint::normalize()
{
    float l = length();
    if (l != 0.0) {
        x /= l;
        y /= l;
        z /= l;
    }
}

inline
void MyQuaternion::rotateX(float angle)
{
    setRotation(angle, MyPoint{1.0f, 0.0f, 0.0f});
}

inline
void MyQuaternion::rotateY(float angle)
{
    MyQuaternion::setRotation(angle, MyPoint{0.0f, 1.0f, 0.0f});
}

inline
void MyQuaternion::rotateZ(float angle)
{
    MyQuaternion::setRotation(angle, MyPoint{0.0f, 0.0f, 1.0f});
}

inline
float MyQuaternion::magnitude() const
{
    return sqrtf(w*w + x*x + y*y + z*z);
}

inline
void MyQuaternion::toOppositeAxis()
{
    x = -x;
    y = -y;
    z = -z;
}

inline
void MyQuaternion::normalize()
{
    const float l = magnitude();
    w /= l;
    x /= l;
    y /= l;
    z /= l;
}

inline
MyQuaternion MyQuaternion::operator*(float mult) const
{
    return MyQuaternion{w*mult, x*mult, y*mult, z*mult};
}

inline
MyQuaternion& MyQuaternion::operator*=(float mult)
{
    w *= mult;
    x *= mult;
    y *= mult;
    z *= mult;
    return *this;
}

inline
MyQuaternion MyQuaternion::operator-(const MyQuaternion& rhs) const
{
    return MyQuaternion{w-rhs.w,x-rhs.x,y-rhs.y,z-rhs.z};
}

inline
MyQuaternion MyQuaternion::operator+(const MyQuaternion& rhs) const
{
    return MyQuaternion{w+rhs.w,x+rhs.x,y+rhs.y,z+rhs.z};
}

inline
MyQuaternion MyQuaternion::operator*(const MyQuaternion& rhs) const
{
    return MyQuaternion{
        -x * rhs.x - y * rhs.y - z * rhs.z + w * rhs.w,
        x * rhs.w + y * rhs.z - z * rhs.y + w * rhs.x,
        -x * rhs.z + y * rhs.w + z * rhs.x + w * rhs.y,
        x * rhs.y - y * rhs.x + z * rhs.w + w * rhs.z
    };
}

inline
float MyQuaternion::dot(const MyQuaternion& rhs) const
{
    return w * rhs.w + x * rhs.x + y * rhs.y + z * rhs.z;
}
