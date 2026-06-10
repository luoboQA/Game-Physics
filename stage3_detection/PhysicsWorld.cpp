#include "PhysicsWorld.h"
#include "GJK.h"
#include "EPA.h"
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
    
    // 4. 精确碰撞检测和响应
    m_contactManifolds.clear();
    const auto& pairs = m_broadphase->GetPotentialCollisions();
    
    for (const auto& pair : pairs) {
        Collider* colliderA = pair.first;
        Collider* colliderB = pair.second;
        
        if (colliderA->body->m_isStatic && colliderB->body->m_isStatic) continue;
        
        GJKResult gjkResult;
        if (GJK(colliderA, colliderB, &gjkResult) && gjkResult.isColliding) {
            ContactManifold manifold;
            if (EPA(colliderA, colliderB, gjkResult.simplex, &manifold)) {
                m_contactManifolds.push_back(manifold);
                
                printf("[PhysicsWorld] COLLISION! Depth: %.4f\n", manifold.penetrationDepth);
                
                // ========== 添加碰撞响应 ==========
                RigidBody* bodyA = colliderA->body;
                RigidBody* bodyB = colliderB->body;
                
                float restitution = 0.5f;  // 弹性系数
                
                // 计算相对速度
                Vec3 rA = manifold.contactPointA - bodyA->m_globalCentroid;
                Vec3 rB = manifold.contactPointB - bodyB->m_globalCentroid;
                
                Vec3 velA = bodyA->m_linearVelocity + bodyA->m_angularVelocity.Cross(rA);
                Vec3 velB = bodyB->m_linearVelocity + bodyB->m_angularVelocity.Cross(rB);
                Vec3 relVel = velB - velA;
                float normalRelVel = relVel.Dot(manifold.normal);
                
                if (normalRelVel < 0) {
                    // 计算冲量
                    float invMassSum = bodyA->m_inverseMass + bodyB->m_inverseMass;
                    
                    Vec3 rA_cross_n = rA.Cross(manifold.normal);
                    Vec3 rB_cross_n = rB.Cross(manifold.normal);
                    float invInertiaA = rA_cross_n.Dot(bodyA->m_globalInverseInertiaTensor * rA_cross_n);
                    float invInertiaB = rB_cross_n.Dot(bodyB->m_globalInverseInertiaTensor * rB_cross_n);
                    float K = invMassSum + invInertiaA + invInertiaB;
                    
                    if (K > 1e-6f) {
                        float j = -(1.0f + restitution) * normalRelVel / K;
                        Vec3 impulse = manifold.normal * j;
                        
                        if (!bodyA->m_isStatic) {
                            bodyA->m_linearVelocity -= impulse * bodyA->m_inverseMass;
                            bodyA->m_angularVelocity -= bodyA->m_globalInverseInertiaTensor * rA.Cross(impulse);
                        }
                        if (!bodyB->m_isStatic) {
                            bodyB->m_linearVelocity += impulse * bodyB->m_inverseMass;
                            bodyB->m_angularVelocity += bodyB->m_globalInverseInertiaTensor * rB.Cross(impulse);
                        }
                    }
                }
                
                // 位置修正
                float percent = 0.2f;
                float slop = 0.01f;
                float correction = std::max(manifold.penetrationDepth - slop, 0.0f) * percent;
                Vec3 correctionVec = manifold.normal * correction;
                
                if (!bodyA->m_isStatic) {
                    bodyA->m_position -= correctionVec;
                    bodyA->UpdateGlobalCentroidFromPosition();
                }
                if (!bodyB->m_isStatic) {
                    bodyB->m_position += correctionVec;
                    bodyB->UpdateGlobalCentroidFromPosition();
                }
            }
        }
    }
    
    // 5. 简单的地面边界保护（可选）
    for (RigidBody* body : m_bodies) {
        if (!body->m_isStatic) {
            if (body->m_position.y < -4.8f) {
                body->m_position.y = -4.8f;
                body->UpdateGlobalCentroidFromPosition();
                if (body->m_linearVelocity.y < 0) {
                    body->m_linearVelocity.y = -body->m_linearVelocity.y * 0.5f;
                }
            }
        }
    }
}