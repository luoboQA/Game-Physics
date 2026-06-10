#ifndef COLLIDER_H
#define COLLIDER_H

#include "AABB.h"

struct RigidBody;

enum ColliderType {
    COLLIDER_BOX,
    COLLIDER_SPHERE,
    COLLIDER_PLANE,
    COLLIDER_POLYHEDRON 
};

struct Collider {
    RigidBody* body;
    ColliderType type;
    Vec3 localCentroid;
    AABB localAABB;
    
    Collider();
    Collider(RigidBody* b, ColliderType t, const Vec3& lc, const AABB& laabb);
    virtual ~Collider() {}
    
    virtual AABB GetWorldAABB() const;
    Vec3 GetWorldCentroid() const;
    void SetBody(RigidBody* b) { body = b; }
    
    virtual Vec3 SupportLocal(const Vec3& direction) const = 0;
    Vec3 SupportWorld(const Vec3& direction) const;
};

// 盒子碰撞体
struct BoxCollider : public Collider {
    Vec3 halfExtents;
    
    BoxCollider(RigidBody* b, const Vec3& lc, const Vec3& halfExtents);
    virtual Vec3 SupportLocal(const Vec3& direction) const override;
};

// 球体碰撞体
struct SphereCollider : public Collider {
    float radius;
    
    SphereCollider(RigidBody* b, const Vec3& lc, float radius);
    virtual Vec3 SupportLocal(const Vec3& direction) const override;
};

#endif