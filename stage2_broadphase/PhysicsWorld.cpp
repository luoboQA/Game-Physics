#include "PhysicsWorld.h"
#include <cstdio>
#include <algorithm>

PhysicsWorld::PhysicsWorld(const Vec3& gravity)
    : m_gravity(gravity) {
    m_broadphase = new NSquaredBroadphase();
    printf("[PhysicsWorld] Created with gravity (%.2f,%.2f,%.2f)\n", 
           gravity.x, gravity.y, gravity.z);
}

PhysicsWorld::~PhysicsWorld() {
    delete m_broadphase;
    for (RigidBody* body : m_bodies) {
        delete body;
    }
}

void PhysicsWorld::AddRigidBody(RigidBody* body) {
    if (body) {
        m_bodies.push_back(body);
        for (Collider* collider : body->m_colliders) {
            m_broadphase->AddCollider(collider);
        }
        printf("[PhysicsWorld] Added rigid body, total: %zu\n", m_bodies.size());
    }
}

void PhysicsWorld::RemoveRigidBody(RigidBody* body) {
    auto it = std::find(m_bodies.begin(), m_bodies.end(), body);
    if (it != m_bodies.end()) {
        for (Collider* collider : body->m_colliders) {
            m_broadphase->RemoveCollider(collider);
        }
        m_bodies.erase(it);
        printf("[PhysicsWorld] Removed rigid body\n");
    }
}

void PhysicsWorld::Step(float dt) {
    // 1. 应用重力
    for (RigidBody* body : m_bodies) {
        if (!body->m_isStatic) {
            Vec3 gravityForce = m_gravity * body->m_mass;
            body->ApplyForce(gravityForce, body->m_globalCentroid);
        }
    }
    
    // 2. 积分更新
    for (RigidBody* body : m_bodies) {
        body->Integrate(dt);
    }
    
    // 3. 更新宽阶段
    m_broadphase->Update();
    
    // 4. 简单的碰撞响应：防止穿透地面
    for (RigidBody* body : m_bodies) {
        if (!body->m_isStatic) {
            // 检查是否穿透地面（地面在 y = -4.5）
            if (body->m_position.y < -4.5f) {
                body->m_position.y = -4.5f;
                body->UpdateGlobalCentroidFromPosition();
                if (body->m_linearVelocity.y < 0) {
                    body->m_linearVelocity.y = -body->m_linearVelocity.y * 0.5f;
                }
            }
        }
    }
    
    // 5. 获取可能碰撞的碰撞体对（仅调试输出）
    const auto& pairs = m_broadphase->GetPotentialCollisions();
    
    static int frameCount = 0;
    frameCount++;
    if (frameCount % 60 == 0 && !pairs.empty()) {
        printf("[PhysicsWorld] %zu potential collision pairs detected\n", pairs.size());
    }
}