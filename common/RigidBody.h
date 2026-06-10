#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "Vec3.h"
#include "Mat3.h"
#include <vector>

struct Collider;

struct RigidBody {
    // 物理属性
    float m_mass;
    float m_inverseMass;
    Mat3 m_localInverseInertiaTensor;
    Mat3 m_globalInverseInertiaTensor;
    
    // 运动状态
    Vec3 m_position;
    Mat3 m_orientation;
    Mat3 m_inverseOrientation;
    Vec3 m_linearVelocity;
    Vec3 m_angularVelocity;
    
    // 力与扭矩累积器
    Vec3 m_forceAccumulator;
    Vec3 m_torqueAccumulator;
    
    // 质心
    Vec3 m_localCentroid;
    Vec3 m_globalCentroid;
    
    // 碰撞体列表
    std::vector<Collider*> m_colliders;
    
    // 静态刚体标志
    bool m_isStatic;
    
    RigidBody(float mass, const Mat3& localInertiaTensor, const Vec3& localCentroid = Vec3());
    ~RigidBody();
    
    // 坐标转换
    Vec3 LocalToGlobal(const Vec3& p) const;
    Vec3 GlobalToLocal(const Vec3& p) const;
    Vec3 LocalToGlobalVec(const Vec3& v) const;
    Vec3 GlobalToLocalVec(const Vec3& v) const;
    
    // 更新方法
    void UpdateGlobalCentroidFromPosition();
    void UpdatePositionFromGlobalCentroid();
    void UpdateGlobalInertiaTensor();
    void UpdateOrientationAndInertia();
    
    // 力应用
    void ApplyForce(const Vec3& force, const Vec3& applicationPointWorld);
    
    // 积分
    void Integrate(float dt);
    
    // 碰撞体管理
    void AddCollider(Collider* collider);
    void RemoveCollider(Collider* collider);
    
    void Print() const;
};

#endif