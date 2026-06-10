#ifndef VEC3_H
#define VEC3_H

#include <cmath>
#include <cstdio>

struct Vec3 {
    float x, y, z;
    
    Vec3();
    Vec3(float x, float y, float z);
    
    // 算术运算符
    Vec3 operator+(const Vec3& v) const;
    Vec3 operator-(const Vec3& v) const;
    Vec3 operator*(float s) const;
    Vec3 operator-() const;
    
    // 复合赋值运算符
    Vec3& operator+=(const Vec3& v);
    Vec3& operator-=(const Vec3& v);
    Vec3& operator*=(float s);
    Vec3& operator/=(float s);
    
    // 向量运算
    float Dot(const Vec3& v) const;
    Vec3 Cross(const Vec3& v) const;
    float Length() const;
    float LengthSq() const;
    Vec3 Normalized() const;
    
    // 修改操作
    void Zero();
    void Negate();
    
    // 调试
    void Print() const;
};

// 非成员运算符
Vec3 operator*(float s, const Vec3& v);

#endif