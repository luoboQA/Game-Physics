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
    std::vector<ContactConstraint> m_previousConstraints;  // 上一帧的约束缓存（用于热启动）
    int m_numIterations;        // 顺序冲量迭代次数
    bool m_enableFriction;      // 是否启用摩擦
    float m_defaultRestitution; // 默认弹性系数
    float m_defaultFriction;    // 默认摩擦系数
    
    // 稳定性参数 (Slops)
    float m_penetrationSlop;     // 穿透容差
    float m_restitutionSlop;     // 速度容差
    bool m_enableWarmStarting;   // 是否启用热启动
    float m_warmStartFactor;     // 热启动因子
    
public:
    PhysicsWorld(const Vec3& gravity = Vec3(0, -9.8f, 0));
    ~PhysicsWorld();
    
    void AddRigidBody(RigidBody* body);
    void RemoveRigidBody(RigidBody* body);
    
    void Step(float dt);
    
    // 约束求解
    void GenerateConstraints();
    void TryMatchPreviousConstraint(ContactConstraint& newConstraint, 
                                     const ContactManifold& manifold);
    void SolveConstraints(float dt);
    void ApplyPositionalCorrection();
    
    // 设置物理参数
    void SetGravity(const Vec3& gravity) { m_gravity = gravity; }
    void SetNumIterations(int iterations) { m_numIterations = iterations; }
    void EnableFriction(bool enable) { m_enableFriction = enable; }
    void SetDefaultRestitution(float restitution) { m_defaultRestitution = restitution; }
    void SetDefaultFriction(float friction) { m_defaultFriction = friction; }
    
    // 稳定性设置
    void SetPenetrationSlop(float slop) { m_penetrationSlop = slop; }
    void SetRestitutionSlop(float slop) { m_restitutionSlop = slop; }
    void EnableWarmStarting(bool enable) { m_enableWarmStarting = enable; }
    void SetWarmStartFactor(float factor) { m_warmStartFactor = factor; }
    
    // 查询
    Broadphase* GetBroadphase() { return m_broadphase; }
    const std::vector<ContactConstraint>& GetConstraints() const { return m_constraints; }
};

#endif