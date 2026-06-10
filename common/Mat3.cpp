#include "Mat3.h"
#include <cassert>
#include <cstdio>

Mat3::Mat3() {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m[i][j] = 0;
}

Vec3 Mat3::operator*(const Vec3& v) const {
    return Vec3(
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
    );
}

Mat3 Mat3::operator*(const Mat3& other) const {
    Mat3 res;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res.m[i][j] = m[i][0] * other.m[0][j] +
                          m[i][1] * other.m[1][j] +
                          m[i][2] * other.m[2][j];
        }
    }
    return res;
}

Mat3 Mat3::Transposed() const {
    Mat3 res;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            res.m[i][j] = m[j][i];
    return res;
}

Mat3 Mat3::Inverted() const {
    Mat3 inv;
    float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    assert(fabs(det) > 1e-6f);
    
    float invDet = 1.0f / det;
    inv.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet;
    inv.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet;
    inv.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;
    inv.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet;
    inv.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
    inv.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invDet;
    inv.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet;
    inv.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * invDet;
    inv.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet;
    return inv;
}

Mat3 Mat3::Identity() {
    Mat3 res;
    res.m[0][0] = 1;
    res.m[1][1] = 1;
    res.m[2][2] = 1;
    return res;
}

Mat3 Mat3::Zero() {
    Mat3 res;
    return res;
}

Mat3 Mat3::RotationAxisAngle(const Vec3& axis, float angle) {
    Vec3 a = axis.Normalized();
    float c = cosf(angle), s = sinf(angle);
    float oc = 1.0f - c;
    
    Mat3 res;
    res.m[0][0] = c + a.x * a.x * oc;
    res.m[0][1] = a.x * a.y * oc - a.z * s;
    res.m[0][2] = a.x * a.z * oc + a.y * s;
    res.m[1][0] = a.y * a.x * oc + a.z * s;
    res.m[1][1] = c + a.y * a.y * oc;
    res.m[1][2] = a.y * a.z * oc - a.x * s;
    res.m[2][0] = a.z * a.x * oc - a.y * s;
    res.m[2][1] = a.z * a.y * oc + a.x * s;
    res.m[2][2] = c + a.z * a.z * oc;
    return res;
}

void Mat3::Print() const {
    for (int i = 0; i < 3; i++) {
        printf("[%.2f, %.2f, %.2f]\n", m[i][0], m[i][1], m[i][2]);
    }
}