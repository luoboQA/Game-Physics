#include "AABB.h"
#include <cstdio>
#include <algorithm>

AABB::AABB() : min(0,0,0), max(0,0,0) {}

AABB::AABB(const Vec3& min_, const Vec3& max_) : min(min_), max(max_) {}

AABB AABB::FromPoints(const Vec3* points, int count) {
    if (count <= 0) return AABB();
    
    Vec3 pMin = points[0];
    Vec3 pMax = points[0];
    
    for (int i = 1; i < count; ++i) {
        pMin.x = std::min(pMin.x, points[i].x);
        pMin.y = std::min(pMin.y, points[i].y);
        pMin.z = std::min(pMin.z, points[i].z);
        pMax.x = std::max(pMax.x, points[i].x);
        pMax.y = std::max(pMax.y, points[i].y);
        pMax.z = std::max(pMax.z, points[i].z);
    }
    
    return AABB(pMin, pMax);
}

bool AABB::Intersects(const AABB& other) const {
    return (min.x <= other.max.x && max.x >= other.min.x) &&
           (min.y <= other.max.y && max.y >= other.min.y) &&
           (min.z <= other.max.z && max.z >= other.min.z);
}

bool AABB::Contains(const Vec3& point) const {
    return (point.x >= min.x && point.x <= max.x) &&
           (point.y >= min.y && point.y <= max.y) &&
           (point.z >= min.z && point.z <= max.z);
}

Vec3 AABB::Center() const {
    return (min + max) * 0.5f;
}

Vec3 AABB::HalfExtents() const {
    return (max - min) * 0.5f;
}

AABB AABB::Union(const AABB& other) const {
    return AABB(
        Vec3(std::min(min.x, other.min.x),
             std::min(min.y, other.min.y),
             std::min(min.z, other.min.z)),
        Vec3(std::max(max.x, other.max.x),
             std::max(max.y, other.max.y),
             std::max(max.z, other.max.z))
    );
}

void AABB::Expand(float margin) {
    min.x -= margin;
    min.y -= margin;
    min.z -= margin;
    max.x += margin;
    max.y += margin;
    max.z += margin;
}

float AABB::Volume() const {
    Vec3 ext = HalfExtents();
    return 8.0f * ext.x * ext.y * ext.z;
}

void AABB::Print() const {
    printf("AABB: min(%.2f,%.2f,%.2f) max(%.2f,%.2f,%.2f)\n",
           min.x, min.y, min.z, max.x, max.y, max.z);
}