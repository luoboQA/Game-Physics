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
    
    // 约束求解器相关
    std::vector<ContactConstraint> m_constraints;
    std::vector<ContactConstraint> m_previousConstraints;
    int m_numIterations;
    bool m_enableFriction;
    float m_defaultRestitution;
    float m_defaultFriction;
    
    // 稳定性参数
    float m_penetrationSlop;
    float m_restitutionSlop;
    bool m_enableWarmStarting;
    float m_warmStartFactor;
    
public:
    PhysicsWorld(const Vec3& gravity = Vec3(0, -9.8f, 0));
    ~PhysicsWorld();
    
    void AddRigidBody(RigidBody* body);
    void RemoveRigidBody(RigidBody* body);
    
    void Step(float dt);
    
    void GenerateConstraints();
    void TryMatchPreviousConstraint(ContactConstraint& newConstraint, 
                                     const ContactManifold& manifold);
    void SolveConstraints(float dt);
    void ApplyPositionalCorrection();
    
    void SetGravity(const Vec3& gravity) { m_gravity = gravity; }
    void SetNumIterations(int iterations) { m_numIterations = iterations; }
    void EnableFriction(bool enable) { m_enableFriction = enable; }
    void SetDefaultRestitution(float restitution) { m_defaultRestitution = restitution; }
    void SetDefaultFriction(float friction) { m_defaultFriction = friction; }
    void SetPenetrationSlop(float slop) { m_penetrationSlop = slop; }
    void SetRestitutionSlop(float slop) { m_restitutionSlop = slop; }
    void EnableWarmStarting(bool enable) { m_enableWarmStarting = enable; }
    void SetWarmStartFactor(float factor) { m_warmStartFactor = factor; }
    
    Broadphase* GetBroadphase() { return m_broadphase; }
    const std::vector<ContactConstraint>& GetConstraints() const { return m_constraints; }
};

#endif