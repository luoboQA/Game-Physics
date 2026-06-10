#ifndef MAT3_H
#define MAT3_H

#include "Vec3.h"

struct Mat3 {
    float m[3][3];
    
    Mat3();
    
    Vec3 operator*(const Vec3& v) const;
    Mat3 operator*(const Mat3& other) const;
    
    Mat3 Transposed() const;
    Mat3 Inverted() const;
    
    static Mat3 Identity();
    static Mat3 Zero();
    static Mat3 RotationAxisAngle(const Vec3& axis, float angle);
    
    void Print() const;
};

#endif