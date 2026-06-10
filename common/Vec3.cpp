#include "Vec3.h"

Vec3::Vec3() : x(0), y(0), z(0) {}

Vec3::Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

Vec3 Vec3::operator+(const Vec3& v) const {
    return Vec3(x + v.x, y + v.y, z + v.z);
}

Vec3 Vec3::operator-(const Vec3& v) const {
    return Vec3(x - v.x, y - v.y, z - v.z);
}

Vec3 Vec3::operator*(float s) const {
    return Vec3(x * s, y * s, z * s);
}

Vec3 operator*(float s, const Vec3& v) {
    return v * s;
}

Vec3 Vec3::operator-() const {
    return Vec3(-x, -y, -z);
}

Vec3& Vec3::operator+=(const Vec3& v) {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
}

Vec3& Vec3::operator-=(const Vec3& v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
}

Vec3& Vec3::operator*=(float s) {
    x *= s;
    y *= s;
    z *= s;
    return *this;
}

Vec3& Vec3::operator/=(float s) {
    if (s != 0) {
        x /= s;
        y /= s;
        z /= s;
    }
    return *this;
}

float Vec3::Dot(const Vec3& v) const {
    return x * v.x + y * v.y + z * v.z;
}

Vec3 Vec3::Cross(const Vec3& v) const {
    return Vec3(
        y * v.z - z * v.y,
        z * v.x - x * v.z,
        x * v.y - y * v.x
    );
}

float Vec3::Length() const {
    return sqrtf(Dot(*this));
}

float Vec3::LengthSq() const {
    return x * x + y * y + z * z;
}

Vec3 Vec3::Normalized() const {
    float l = Length();
    if (l > 1e-6f)
        return *this * (1.0f / l);
    return *this;
}

void Vec3::Zero() {
    x = y = z = 0;
}

void Vec3::Negate() {
    x = -x;
    y = -y;
    z = -z;
}

void Vec3::Print() const {
    printf("(%.2f, %.2f, %.2f)", x, y, z);
}