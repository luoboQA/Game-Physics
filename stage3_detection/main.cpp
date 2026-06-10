#include <cstdio>
#include <cmath>
#include <vector>
#include <algorithm>

#include "RigidBody.h"
#include "Collider.h"
#include "PhysicsWorld.h"
#include "Broadphase.h"
#include "GJK.h"
#include "EPA.h"

// 创建盒子刚体的辅助函数
RigidBody* CreateBox(const Vec3& position, const Vec3& halfExtents, 
                     float mass, const Vec3& velocity = Vec3(0,0,0)) {
    float width = halfExtents.x * 2;
    float height = halfExtents.y * 2;
    float depth = halfExtents.z * 2;
    
    float inertiaXX = (1.0f / 12.0f) * mass * (height * height + depth * depth);
    float inertiaYY = (1.0f / 12.0f) * mass * (width * width + depth * depth);
    float inertiaZZ = (1.0f / 12.0f) * mass * (width * width + height * height);
    
    Mat3 inertiaTensor;
    inertiaTensor.m[0][0] = inertiaXX;
    inertiaTensor.m[1][1] = inertiaYY;
    inertiaTensor.m[2][2] = inertiaZZ;
    
    RigidBody* body = new RigidBody(mass, inertiaTensor);
    body->m_position = position;
    body->m_linearVelocity = velocity;
    body->m_orientation = Mat3::Identity();
    body->UpdateOrientationAndInertia();
    body->UpdateGlobalCentroidFromPosition();
    
    BoxCollider* collider = new BoxCollider(body, Vec3(0,0,0), halfExtents);
    body->AddCollider(collider);
    
    return body;
}

// 创建球体刚体的辅助函数
RigidBody* CreateSphere(const Vec3& position, float radius, 
                        float mass, const Vec3& velocity = Vec3(0,0,0)) {
    float inertiaValue = (2.0f / 5.0f) * mass * radius * radius;
    Mat3 inertiaTensor;
    inertiaTensor.m[0][0] = inertiaValue;
    inertiaTensor.m[1][1] = inertiaValue;
    inertiaTensor.m[2][2] = inertiaValue;
    
    RigidBody* body = new RigidBody(mass, inertiaTensor);
    body->m_position = position;
    body->m_linearVelocity = velocity;
    body->m_orientation = Mat3::Identity();
    body->UpdateOrientationAndInertia();
    body->UpdateGlobalCentroidFromPosition();
    
    SphereCollider* collider = new SphereCollider(body, Vec3(0,0,0), radius);
    body->AddCollider(collider);
    
    return body;
}

int main() {
    printf("========================================\n");
    printf("Stage 3: GJK + EPA Collision Detection\n");
    printf("========================================\n\n");
    
    PhysicsWorld world(Vec3(0, -9.8f, 0));
    
    // 创建地面（静态刚体）
    RigidBody* ground = CreateBox(Vec3(0, -5, 0), Vec3(10, 0.5f, 10), 0.0f);
    world.AddRigidBody(ground);
    printf("Ground: position y=%.2f, top at y=%.2f\n\n", 
           ground->m_position.y, ground->m_position.y + 0.5f);
    
    // 创建下落的立方体
    RigidBody* cube = CreateBox(Vec3(0, 5, 0), Vec3(0.5f, 0.5f, 0.5f), 1.0f);
    world.AddRigidBody(cube);
    
    // 创建下落的球体
    RigidBody* sphere = CreateSphere(Vec3(1.5f, 5, 0), 0.5f, 1.0f);
    world.AddRigidBody(sphere);
    
    printf("Initial states:\n");
    cube->Print();
    sphere->Print();
    printf("\n");
    
    // 模拟 2 秒
    float dt = 1.0f / 60.0f;
    for (int step = 0; step < 120; ++step) {
        world.Step(dt);
        
        if (step % 30 == 0) {
            printf("\n--- Time: %.2f seconds ---\n", step * dt);
            cube->Print();
            sphere->Print();
            
            const auto& manifolds = world.GetContactManifolds();
            if (!manifolds.empty()) {
                printf("Contact manifolds: %zu\n", manifolds.size());
                for (const auto& m : manifolds) {
                    printf("  Penetration: %.4f, Normal: (%.2f,%.2f,%.2f)\n",
                           m.penetrationDepth, m.normal.x, m.normal.y, m.normal.z);
                }
            }
        }
    }
    
    printf("\n=== Simulation Complete ===\n");
    
    return 0;
}