#ifndef PHYSICS_WORLD_H
#define PHYSICS_WORLD_H

#include "RigidBody.h"
#include "Broadphase.h"
#include "EPA.h"
#include <vector>

class PhysicsWorld {
private:
    std::vector<RigidBody*> m_bodies;
    Broadphase* m_broadphase;
    Vec3 m_gravity;
    std::vector<ContactManifold> m_contactManifolds;
    
public:
    PhysicsWorld(const Vec3& gravity = Vec3(0, -9.8f, 0));
    ~PhysicsWorld();
    
    void AddRigidBody(RigidBody* body);
    void RemoveRigidBody(RigidBody* body);
    
    void Step(float dt);
    
    Broadphase* GetBroadphase() { return m_broadphase; }
    void SetGravity(const Vec3& gravity) { m_gravity = gravity; }
    
    const std::vector<ContactManifold>& GetContactManifolds() const { 
        return m_contactManifolds; 
    }
};

#endif