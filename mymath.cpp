#include "mymath.h"

#include <cstdio>
#include <cmath>
#include <cstring>

using namespace std;

void MyMatrix::print()
{
    printf("[ %3.2f %3.2f %3.2f %3.2f ]\n"
           "[ %3.2f %3.2f %3.2f %3.2f ]\n"
           "[ %3.2f %3.2f %3.2f %3.2f ]\n"
           "[ %3.2f %3.2f %3.2f %3.2f ]\n",
           get(0, 0), get(0, 1), get(0, 2), get(0, 3),
           get(1, 0), get(1, 1), get(1, 2), get(1, 3),
           get(2, 0), get(2, 1), get(2, 2), get(2, 3),
           get(3, 0), get(3, 1), get(3, 2), get(3, 3));
}

void MyMatrix::reset()
{
    MyMatrix n;
    memcpy(buf,n.buf, sizeof(buf));
}

MyMatrix& MyMatrix::rotateX(const double angleInRad)
{
    set(1, 1, cosf(angleInRad));
    set(1, 2, -sinf(angleInRad));
    set(2, 2, get(1, 1));
    set(2, 1, -get(1, 2));
    return *this;
}

MyMatrix& MyMatrix::rotateZ(const double angleInRad)
{
    set(0, 0, cosf(angleInRad));
    set(0, 1, -sinf(angleInRad));
    set(1, 1, get(0, 0));
    set(1, 0, -get(0, 1));
    return *this;
}

MyMatrix& MyMatrix::rotateY(const double angleInRad)
{
    set(0, 0, cosf(angleInRad));
    set(0, 2, sinf(angleInRad));
    set(2, 0, -get(0, 2));
    set(2, 2, get(0, 0));
    return *this;
}

MyMatrix MyMatrix::operator*(const MyMatrix rhs) const
{
    MyMatrix ret;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            ret.set(i, j, 0.0f);
            for (int k = 0; k < 4; ++k) {
                ret.set(i, j, ret.get(i,j) + get(i, k) * rhs.get(k, j));
            }
        }
    }
    return ret;
}

MyMatrix MyMatrix::transpose() const
{
    MyMatrix ret;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            ret.set(i, j, get(j, i));
        }
    }
    return ret;
}

void MyPoint::print() const
{
    printf("%.3f %.3f %.3f\n", x, y, z);
}

MyPoint MyPoint::transform(const MyMatrix& m) const
{
    MyPoint ret;
    ret.x = x*m.get(0,0)+y*m.get(0, 1)+z*m.get(0,2)+m.get(0,3);
    ret.y = x*m.get(1,0)+y*m.get(1, 1)+z*m.get(1,2)+m.get(1,3);
    ret.z = x*m.get(2,0)+y*m.get(2, 1)+z*m.get(2,2)+m.get(2,3);
    //print();
    //ret.print();
    return ret;
}

MyPoint MyPoint::transform(const MyQuaternion& q) const
{
    MyQuaternion v(0.0f, x, y, z);
    MyQuaternion conj(q);
    conj.toOppositeAxis();
    v = q * v * conj;
    return MyPoint(v.x, v.y, v.z);
}

void MyQuaternion::setRotation(float angle, MyPoint axis)
{
    w = cosf(angle / 2);
    float s = sinf(angle / 2);
    axis.normalize();
    x = axis.x * s;
    y = axis.y * s;
    z = axis.z * s;
}


MyMatrix MyQuaternion::toMatrix() const
{
    MyQuaternion q = *this;
    q.normalize();

    MyMatrix ret;
    ret.set(0, 0, 1.0f - 2.0f * y * y - 2.0f * z * z);
    ret.set(0, 1, 2.0f * x * y - 2.0f * w * z);
    ret.set(0, 2, 2.0f * x * z + 2.0f * w * y);
    ret.set(1, 0, 2.0f * x * y + 2.0f * w * z);
    ret.set(1, 1, 1.0f - 2.0f * x * x - 2.0f * z * z);
    ret.set(1, 2, 2.0f * y * z - 2.0f * w * x);
    ret.set(2, 0, 2.0f * x * z - 2.0f * w * y);
    ret.set(2, 1, 2.0f * y * z + 2.0f * w * x);
    ret.set(2, 2, 1.0f - 2.0f * x * x - 2.0f * y * y);

    return ret;
}

MyQuaternion
MyQuaternion::slerp(const MyQuaternion& start, MyQuaternion end, float t)
{
    float dot = start.dot(end);
    if (dot < 0) {
        dot = fabs(dot);
        end *= -1.0f;
    }

    if (dot > 0.95) {
        MyQuaternion ret = start + (end - start) * t;
        ret.normalize();
        return ret;
    }
    const float finalAngle = acosf(dot);
    const float angle = finalAngle * t;
    const float fact0 = cosf(angle) - dot * sinf(angle) / sinf(finalAngle);
    const float fact1 = sinf(angle) / sinf(finalAngle);
    return start * fact0 + end * fact1;
}

