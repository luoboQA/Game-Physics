#ifndef AABB_H
#define AABB_H

#include "Vec3.h"

struct AABB {
    Vec3 min;
    Vec3 max;
    
    AABB();
    AABB(const Vec3& min_, const Vec3& max_);
    
    static AABB FromPoints(const Vec3* points, int count);
    
    bool Intersects(const AABB& other) const;
    bool Contains(const Vec3& point) const;
    
    Vec3 Center() const;
    Vec3 HalfExtents() const;
    
    AABB Union(const AABB& other) const;
    void Expand(float margin);
    float Volume() const;
    
    void Print() const;
};

#endif