#ifndef PHYSICS_WORLD_H
#define PHYSICS_WORLD_H

#include "RigidBody.h"
#include "Broadphase.h"
#include "ContactConstraint.h"
#include "EPA.h"
#include <vector>

class PhysicsWorld {
private:
    std::vector<RigidBody*> m_bodies;
    Broadphase* m_broadphase;
    Vec3 m_gravity;
    
    std::vector<ContactConstraint> m_constraints;
    int m_numIterations;
    bool m_enableFriction;
    float m_defaultRestitution;
    float m_defaultFriction;
    
public:
    PhysicsWorld(const Vec3& gravity = Vec3(0, -9.8f, 0));
    ~PhysicsWorld();
    
    void AddRigidBody(RigidBody* body);
    void RemoveRigidBody(RigidBody* body);
    void Step(float dt);
    
    void GenerateConstraints();
    void SolveConstraints(float dt);
    void ApplyPositionalCorrection();
    
    void SetGravity(const Vec3& gravity) { m_gravity = gravity; }
    void SetNumIterations(int iterations) { m_numIterations = iterations; }
    void EnableFriction(bool enable) { m_enableFriction = enable; }
    void SetDefaultRestitution(float restitution) { m_defaultRestitution = restitution; }
    void SetDefaultFriction(float friction) { m_defaultFriction = friction; }
    
    Broadphase* GetBroadphase() { return m_broadphase; }
};

#endif