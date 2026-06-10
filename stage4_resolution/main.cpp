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
    printf("Stage 4: Collision Resolution\n");
    printf("Sequential Impulse + Friction\n");
    printf("========================================\n\n");
    
    // 创建物理世界
    PhysicsWorld world(Vec3(0, -9.8f, 0));
    
    // 配置参数
    world.SetNumIterations(10);        // 更多迭代获得更好稳定性
    world.SetDefaultRestitution(0.5f); // 弹性系数
    world.SetDefaultFriction(0.5f);    // 摩擦系数
    world.EnableFriction(true);
    
    // 地面
    RigidBody* ground = CreateBox(Vec3(0, -5, 0), Vec3(10, 0.5f, 10), 0.0f);
    world.AddRigidBody(ground);
    printf("Ground: position y=%.2f, top at y=%.2f\n\n", 
           ground->m_position.y, ground->m_position.y + 0.5f);
    
    // 测试物体
    std::vector<RigidBody*> testBodies;
    
    // 立方体从 y=3 落下
    testBodies.push_back(CreateBox(Vec3(0, 3, 0), Vec3(0.5f, 0.5f, 0.5f), 1.0f));
    
    // 球体从 y=3 落下
    testBodies.push_back(CreateSphere(Vec3(1.5f, 3, 0), 0.5f, 1.0f));
    
    // 堆叠测试 - 两个盒子堆叠
    testBodies.push_back(CreateBox(Vec3(-1.5f, 0.6f, 0), Vec3(0.4f, 0.4f, 0.4f), 1.0f));
    testBodies.push_back(CreateBox(Vec3(-1.5f, 1.4f, 0), Vec3(0.4f, 0.4f, 0.4f), 1.0f));
    
    for (RigidBody* body : testBodies) {
        world.AddRigidBody(body);
    }
    
    printf("Test objects: Cube, Sphere, Stacked Boxes\n\n");
    
    // 模拟
    float dt = 1.0f / 60.0f;
    
    for (int frame = 0; frame <= 180; ++frame) {
        world.Step(dt);
        
        if (frame % 60 == 0 || frame == 179) {
            printf("\n--- Time: %.2f seconds ---\n", frame * dt);
            
            const char* names[] = {"Cube", "Sphere", "StackBot", "StackTop"};
            for (size_t i = 0; i < testBodies.size(); ++i) {
                Vec3 pos = testBodies[i]->m_position;
                Vec3 vel = testBodies[i]->m_linearVelocity;
                
                float bottom = pos.y - 0.5f;
                if (i == 1) bottom = pos.y - 0.5f;  // 球体半径0.5
                
                printf("%s: y=%.2f (bottom=%.2f), vy=%.2f\n", 
                       names[i], pos.y, bottom, vel.y);
            }
        }
    }
    
    printf("\n=== Simulation Complete ===\n");
    
    return 0;
}