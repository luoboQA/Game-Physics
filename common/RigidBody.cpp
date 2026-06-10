#include "RigidBody.h"
#include "Collider.h"
#include <cstdio>
#include <algorithm>

RigidBody::RigidBody(float mass, const Mat3& localInertiaTensor, const Vec3& localCentroid)
    : m_mass(mass), 
      m_localCentroid(localCentroid), 
      m_globalCentroid(0,0,0),
      m_isStatic(mass <= 0.0f)
{
    if (m_isStatic) {
        m_inverseMass = 0.0f;
        m_localInverseInertiaTensor = Mat3::Zero();
        m_globalInverseInertiaTensor = Mat3::Zero();
        m_linearVelocity = Vec3(0, 0, 0);
        m_angularVelocity = Vec3(0, 0, 0);
    } else {
        m_inverseMass = 1.0f / mass;
        m_localInverseInertiaTensor = localInertiaTensor.Inverted();
        m_globalInverseInertiaTensor = m_localInverseInertiaTensor;
    }
    
    m_position = Vec3(0, 0, 0);
    m_orientation = Mat3::Identity();
    m_inverseOrientation = Mat3::Identity();
    
    m_forceAccumulator = Vec3(0, 0, 0);
    m_torqueAccumulator = Vec3(0, 0, 0);
    
    UpdateGlobalCentroidFromPosition();
    UpdateGlobalInertiaTensor();
}

RigidBody::~RigidBody() {
    for (Collider* collider : m_colliders) {
        delete collider;
    }
    m_colliders.clear();
}

void RigidBody::UpdateGlobalCentroidFromPosition() {
    m_globalCentroid = m_orientation * m_localCentroid + m_position;
}

void RigidBody::UpdatePositionFromGlobalCentroid() {
    m_position = m_globalCentroid - (m_orientation * m_localCentroid);
}

void RigidBody::UpdateGlobalInertiaTensor() {
    if (m_isStatic) {
        m_globalInverseInertiaTensor = Mat3::Zero();
    } else {
        m_globalInverseInertiaTensor = m_orientation * m_localInverseInertiaTensor * m_inverseOrientation;
    }
}

Vec3 RigidBody::LocalToGlobal(const Vec3& p) const {
    return m_orientation * p + m_position;
}

Vec3 RigidBody::GlobalToLocal(const Vec3& p) const {
    return m_inverseOrientation * (p - m_position);
}

Vec3 RigidBody::LocalToGlobalVec(const Vec3& v) const {
    return m_orientation * v;
}

Vec3 RigidBody::GlobalToLocalVec(const Vec3& v) const {
    return m_inverseOrientation * v;
}

void RigidBody::ApplyForce(const Vec3& force, const Vec3& applicationPointWorld) {
    if (m_isStatic) return;
    
    m_forceAccumulator += force;
    Vec3 r = applicationPointWorld - m_globalCentroid;
    m_torqueAccumulator += r.Cross(force);
}

void RigidBody::UpdateOrientationAndInertia() {
    m_inverseOrientation = m_orientation.Transposed();
    UpdateGlobalInertiaTensor();
}

void RigidBody::Integrate(float dt) {
    if (m_isStatic) return;
    
    // 线速度更新
    Vec3 dv = m_forceAccumulator * (m_inverseMass * dt);
    m_linearVelocity += dv;
    
    // 角速度更新
    Vec3 angularImpulse = m_torqueAccumulator * dt;
    Vec3 dw = m_globalInverseInertiaTensor * angularImpulse;
    m_angularVelocity += dw;
    
    // 清空累积器
    m_forceAccumulator.Zero();
    m_torqueAccumulator.Zero();
    
    // 位置更新
    m_globalCentroid += m_linearVelocity * dt;
    UpdatePositionFromGlobalCentroid();
    
    // 朝向更新（使用角速度）
    float angle = m_angularVelocity.Length() * dt;
    if (angle > 1e-6f) {
        Vec3 axis = m_angularVelocity.Normalized();
        Mat3 deltaRot = Mat3::RotationAxisAngle(axis, angle);
        m_orientation = deltaRot * m_orientation;
        
        // 注意：如果矩阵不是正交的，需要重新正交化
        // 但对于小角度，误差很小
    }
    
    UpdateOrientationAndInertia();
}

void RigidBody::AddCollider(Collider* collider) {
    if (collider) {
        m_colliders.push_back(collider);
        collider->SetBody(this);
    }
}

void RigidBody::RemoveCollider(Collider* collider) {
    auto it = std::find(m_colliders.begin(), m_colliders.end(), collider);
    if (it != m_colliders.end()) {
        m_colliders.erase(it);
    }
}

void RigidBody::Print() const {
    printf("Pos: (%.2f, %.2f, %.2f)  Vel: (%.2f, %.2f, %.2f) %s\n",
           m_position.x, m_position.y, m_position.z,
           m_linearVelocity.x, m_linearVelocity.y, m_linearVelocity.z,
           m_isStatic ? "[STATIC]" : "");
    printf("AngVel: (%.2f, %.2f, %.2f)\n", 
           m_angularVelocity.x, m_angularVelocity.y, m_angularVelocity.z);
}