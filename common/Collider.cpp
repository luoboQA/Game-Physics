#include "Collider.h"
#include "RigidBody.h"
#include <cstdio>

Collider::Collider() 
    : body(nullptr), type(COLLIDER_BOX), localCentroid(0,0,0) {}

Collider::Collider(RigidBody* b, ColliderType t, const Vec3& lc, const AABB& laabb)
    : body(b), type(t), localCentroid(lc), localAABB(laabb) {}

AABB Collider::GetWorldAABB() const {
    if (!body) return localAABB;
    
    Vec3 corners[8];
    corners[0] = Vec3(localAABB.min.x, localAABB.min.y, localAABB.min.z);
    corners[1] = Vec3(localAABB.max.x, localAABB.min.y, localAABB.min.z);
    corners[2] = Vec3(localAABB.min.x, localAABB.max.y, localAABB.min.z);
    corners[3] = Vec3(localAABB.max.x, localAABB.max.y, localAABB.min.z);
    corners[4] = Vec3(localAABB.min.x, localAABB.min.y, localAABB.max.z);
    corners[5] = Vec3(localAABB.max.x, localAABB.min.y, localAABB.max.z);
    corners[6] = Vec3(localAABB.min.x, localAABB.max.y, localAABB.max.z);
    corners[7] = Vec3(localAABB.max.x, localAABB.max.y, localAABB.max.z);
    
    Vec3 worldCorners[8];
    for (int i = 0; i < 8; ++i) {
        worldCorners[i] = body->LocalToGlobal(corners[i]);
    }
    
    return AABB::FromPoints(worldCorners, 8);
}

Vec3 Collider::GetWorldCentroid() const {
    if (!body) return localCentroid;
    return body->LocalToGlobal(localCentroid);
}

Vec3 Collider::SupportWorld(const Vec3& direction) const {
    Vec3 localDir = body->GlobalToLocalVec(direction);
    Vec3 localSupport = SupportLocal(localDir);
    return body->LocalToGlobal(localSupport);
}

BoxCollider::BoxCollider(RigidBody* b, const Vec3& lc, const Vec3& he)
    : Collider(b, COLLIDER_BOX, lc, 
               AABB(Vec3(-he.x, -he.y, -he.z) + lc,
                    Vec3( he.x,  he.y,  he.z) + lc)), 
      halfExtents(he) {}

Vec3 BoxCollider::SupportLocal(const Vec3& direction) const {
    Vec3 dirNorm = direction.Normalized();
    Vec3 support;
    support.x = dirNorm.x > 0 ? halfExtents.x : -halfExtents.x;
    support.y = dirNorm.y > 0 ? halfExtents.y : -halfExtents.y;
    support.z = dirNorm.z > 0 ? halfExtents.z : -halfExtents.z;
    return support + localCentroid;
}

SphereCollider::SphereCollider(RigidBody* b, const Vec3& lc, float r)
    : Collider(b, COLLIDER_SPHERE, lc, 
               AABB(Vec3(-r, -r, -r) + lc,
                    Vec3( r,  r,  r) + lc)), 
      radius(r) {}

Vec3 SphereCollider::SupportLocal(const Vec3& direction) const {
    Vec3 dirNorm = direction.Normalized();
    return dirNorm * radius + localCentroid;
}