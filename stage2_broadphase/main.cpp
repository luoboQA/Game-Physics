#include <cstdio>
#include <cmath>
#include <vector>
#include <algorithm>

#include "RigidBody.h"
#include "Collider.h"
#include "PhysicsWorld.h"
#include "Broadphase.h"

// 创建盒子刚体的辅助函数
RigidBody* CreateBox(const Vec3& position, const Vec3& halfExtents, 
                     float mass, const Vec3& velocity = Vec3(0,0,0)) {
    // 计算惯性张量
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
    
    // 添加盒子碰撞体
    BoxCollider* collider = new BoxCollider(body, Vec3(0,0,0), halfExtents);
    body->AddCollider(collider);
    
    return body;
}

int main() {
    printf("========================================\n");
    printf("Stage 2: Broadphase + AABB Collision\n");
    printf("========================================\n\n");
    
    // 创建物理世界
    PhysicsWorld world(Vec3(0, -9.8f, 0));
    
    // 创建地面（静态刚体）
    RigidBody* ground = CreateBox(Vec3(0, -5, 0), Vec3(10, 0.5f, 10), 0.0f);
    world.AddRigidBody(ground);
    printf("Ground: position y=%.2f, half-height=0.5, top at y=%.2f\n\n", 
           ground->m_position.y, ground->m_position.y + 0.5f);
    
    // 创建下落的立方体（动态刚体）
    RigidBody* cube = CreateBox(Vec3(0, 5, 0), Vec3(0.5f, 0.5f, 0.5f), 1.0f);
    world.AddRigidBody(cube);
    
    printf("Initial states:\n");
    ground->Print();
    cube->Print();
    printf("\n");
    
    // 模拟 2 秒（120帧）
    float dt = 1.0f / 60.0f;
    for (int step = 0; step < 120; ++step) {
        world.Step(dt);
        
        // 每30帧打印一次状态
        if (step % 30 == 0) {
            printf("\n--- Time: %.2f seconds ---\n", step * dt);
            cube->Print();
        }
    }
    
    printf("\n=== Simulation Complete ===\n");
    
    return 0;
}