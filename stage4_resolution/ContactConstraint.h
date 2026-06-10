#ifndef CONTACT_CONSTRAINT_H
#define CONTACT_CONSTRAINT_H

#include "Vec3.h"
#include "RigidBody.h"

// 接触约束（顺序冲量求解器）
class ContactConstraint {
public:
    ContactConstraint();
    ContactConstraint(RigidBody* bodyA, RigidBody* bodyB, 
                     const Vec3& contactPointA, const Vec3& contactPointB,
                     const Vec3& normal, float penetrationDepth);
    
    // 求解约束（计算并应用冲量）
    void Solve(float dt);
    
    // 重置累积冲量（每帧开始前调用）
    void ResetAccumulatedImpulse();
    
    // 位置修正（防止穿透）
    void PositionalCorrection();
    
    // 获取接触信息
    Vec3 GetContactPointA() const { return m_contactPointA; }
    Vec3 GetContactPointB() const { return m_contactPointB; }
    Vec3 GetNormal() const { return m_normal; }
    float GetPenetrationDepth() const { return m_penetrationDepth; }
    
    RigidBody* GetBodyA() const { return m_bodyA; }
    RigidBody* GetBodyB() const { return m_bodyB; }
    
    // 设置参数
    void SetRestitution(float restitution) { m_restitution = restitution; }
    void SetFrictionCoefficient(float friction);
    
private:
    RigidBody* m_bodyA;
    RigidBody* m_bodyB;
    
    Vec3 m_contactPointA;      // 物体A上的接触点（世界空间）
    Vec3 m_contactPointB;      // 物体B上的接触点（世界空间）
    Vec3 m_localContactA;      // 物体A上的接触点（局部空间）
    Vec3 m_localContactB;      // 物体B上的接触点（局部空间）
    Vec3 m_normal;             // 碰撞法线（从A指向B）
    float m_penetrationDepth;  // 穿透深度
    
    float m_restitution;       // 弹性系数
    float m_accumulatedNormalImpulse;  // 累积法向冲量
    
    // 摩擦相关
    float m_frictionCoefficient;  // 摩擦系数
    Vec3 m_tangent1;              // 切向1
    Vec3 m_tangent2;              // 切向2
    float m_accumulatedTangentImpulse1;  // 累积切向冲量1
    float m_accumulatedTangentImpulse2;  // 累积切向冲量2
};

#endif