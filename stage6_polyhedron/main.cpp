#include <cstdio>
#include <cmath>
#include <vector>
#include <array>

#include "RigidBody.h"
#include "Collider.h"
#include "PolyhedronCollider.h"
#include "PhysicsWorld.h"
#include "Broadphase.h"
#include "GJK.h"
#include "EPA.h"

// 创建立方体多面体
RigidBody* CreatePolyhedronCube(const Vec3& position, const Vec3& halfExtents, 
                                 float mass, const Vec3& velocity = Vec3(0,0,0)) {
    // 生成立方体的8个顶点
    std::vector<Vec3> vertices = {
        Vec3(-halfExtents.x, -halfExtents.y, -halfExtents.z),
        Vec3( halfExtents.x, -halfExtents.y, -halfExtents.z),
        Vec3( halfExtents.x, -halfExtents.y,  halfExtents.z),
        Vec3(-halfExtents.x, -halfExtents.y,  halfExtents.z),
        Vec3(-halfExtents.x,  halfExtents.y, -halfExtents.z),
        Vec3( halfExtents.x,  halfExtents.y, -halfExtents.z),
        Vec3( halfExtents.x,  halfExtents.y,  halfExtents.z),
        Vec3(-halfExtents.x,  halfExtents.y,  halfExtents.z)
    };
    
    // 立方体的12个三角形面
    std::vector<int> indices = {
        // 底面
        0,1,2,  0,2,3,
        // 顶面
        4,6,5,  4,7,6,
        // 前面
        0,5,1,  0,4,5,
        // 后面
        3,2,6,  3,6,7,
        // 左面
        0,3,7,  0,7,4,
        // 右面
        1,5,6,  1,6,2
    };
    
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
    
    PolyhedronCollider* collider = new PolyhedronCollider(body, Vec3(0,0,0), vertices, indices);
    body->AddCollider(collider);
    
    return body;
}

// 创建简单盒子（使用 BoxCollider，用于堆叠测试）
RigidBody* CreateSimpleBox(const Vec3& position, const Vec3& halfExtents, 
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

// 创建球体
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

// 创建四面体
RigidBody* CreateTetrahedron(const Vec3& position, float size, 
                              float mass, const Vec3& velocity = Vec3(0,0,0)) {
    float h = size * sqrtf(6.0f) / 4.0f;  // 高度
    
    std::vector<Vec3> vertices = {
        Vec3(0, 0, 0),
        Vec3(size, 0, 0),
        Vec3(size/2, 0, size * sqrtf(3.0f)/2),
        Vec3(size/2, h, size * sqrtf(3.0f)/6)
    };
    
    // 四面体的4个面
    std::vector<int> indices = {
        0,1,2,   // 底面
        0,3,1,   // 侧面1
        1,3,2,   // 侧面2
        2,3,0    // 侧面3
    };
    
    // 简化惯性张量计算
    float approxInertia = 0.1f * mass * size * size;
    Mat3 inertiaTensor;
    inertiaTensor.m[0][0] = approxInertia;
    inertiaTensor.m[1][1] = approxInertia;
    inertiaTensor.m[2][2] = approxInertia;
    
    RigidBody* body = new RigidBody(mass, inertiaTensor);
    body->m_position = position;
    body->m_linearVelocity = velocity;
    body->m_orientation = Mat3::Identity();
    body->UpdateOrientationAndInertia();
    body->UpdateGlobalCentroidFromPosition();
    
    // 重新居中顶点
    Vec3 centroid(0,0,0);
    for (const auto& v : vertices) centroid = centroid + v;
    centroid = centroid * (1.0f / vertices.size());
    
    std::vector<Vec3> centeredVertices;
    for (const auto& v : vertices) centeredVertices.push_back(v - centroid);
    
    PolyhedronCollider* collider = new PolyhedronCollider(body, centroid, centeredVertices, indices);
    body->AddCollider(collider);
    
    return body;
}

// 创建盒子（使用多面体系统，用于演示多面体功能）
RigidBody* CreatePolyhedronBox(const Vec3& position, const Vec3& halfExtents, 
                                float mass, const Vec3& velocity = Vec3(0,0,0)) {
    return CreatePolyhedronCube(position, halfExtents, mass, velocity);
}

int main() {
    printf("========================================\n");
    printf("Stage 6: Polyhedron Collider Support\n");
    printf("GJK + EPA with Arbitrary Convex Shapes\n");
    printf("========================================\n\n");
    
    // 创建物理世界
    PhysicsWorld world(Vec3(0, -9.8f, 0));
    
    // 配置参数
    world.SetNumIterations(20);
    world.SetDefaultRestitution(0.3f);
    world.SetDefaultFriction(0.5f);
    world.EnableFriction(true);
    world.EnableWarmStarting(true);
    world.SetPenetrationSlop(0.005f);
    world.SetRestitutionSlop(0.3f);
    
    // 地面（使用多面体盒子）
    RigidBody* ground = CreatePolyhedronBox(Vec3(0, -5, 0), Vec3(15, 0.5f, 15), 0.0f);
    world.AddRigidBody(ground);
    printf("Ground: top at y=%.2f\n\n", ground->m_position.y + 0.5f);
    
    std::vector<RigidBody*> testBodies;
    
    // 测试1：立方体多面体
    testBodies.push_back(CreatePolyhedronCube(Vec3(0, 2.5f, 0), Vec3(0.5f, 0.5f, 0.5f), 1.0f));
    
    // 测试2：球体
    testBodies.push_back(CreateSphere(Vec3(1.5f, 2.5f, 0), 0.5f, 1.0f));
    
    // 测试3：四面体
    testBodies.push_back(CreateTetrahedron(Vec3(-1.5f, 2.5f, 0), 0.8f, 1.0f));
    
    // 测试4：堆叠测试 - 使用 SimpleBox（BoxCollider）确保堆叠稳定
    printf("\n[Note] Stacked cubes use BoxCollider for stable stacking\n");
    testBodies.push_back(CreateSimpleBox(Vec3(2.0f, 0.6f, 1.5f), Vec3(0.4f, 0.4f, 0.4f), 1.0f));
    testBodies.push_back(CreateSimpleBox(Vec3(2.0f, 1.4f, 1.5f), Vec3(0.4f, 0.4f, 0.4f), 1.0f));
    
    for (RigidBody* body : testBodies) {
        world.AddRigidBody(body);
    }
    
    printf("\nTest objects:\n");
    printf("  - Cube (polyhedron, 8 vertices, 12 triangles)\n");
    printf("  - Sphere (implicit shape)\n");
    printf("  - Tetrahedron (polyhedron, 4 vertices, 4 triangles)\n");
    printf("  - Stacked cubes (2 layers) - using BoxCollider for stability\n\n");
    
    printf("Simulating 3 seconds...\n\n");
    
    // 模拟
    float dt = 1.0f / 60.0f;
    
    for (int frame = 0; frame <= 180; ++frame) {
        world.Step(dt);
        
        if (frame % 60 == 0 || frame == 179) {
            printf("\n--- Time: %.2f seconds ---\n", frame * dt);
            
            const char* names[] = {"Cube", "Sphere", "Tetra", "StackBot", "StackTop"};
            for (size_t i = 0; i < testBodies.size(); ++i) {
                Vec3 pos = testBodies[i]->m_position;
                Vec3 vel = testBodies[i]->m_linearVelocity;
                
                float bottom = pos.y - 0.5f;
                if (i == 1) bottom = pos.y - 0.5f;  // 球体半径0.5
                if (i == 2) bottom = pos.y - 0.4f;  // 四面体近似
                
                printf("%s: y=%.2f (bottom=%.2f), vy=%.2f\n", 
                       names[i], pos.y, bottom, vel.y);
            }
        }
        
        // 检查是否所有物体都已稳定
        bool allSettled = true;
        for (auto body : testBodies) {
            if (!body->m_isStatic && fabs(body->m_linearVelocity.y) > 0.1f) {
                allSettled = false;
                break;
            }
        }
        
        if (allSettled && frame > 60) {
            printf("\n=== All bodies settled at frame %d ===\n", frame);
            break;
        }
    }
    
    printf("\n=== Simulation Complete ===\n");
    printf("\nKey features demonstrated:\n");
    printf("  - GJK/EPA works with polyhedra via SupportLocal()\n");
    printf("  - Half-edge mesh provides O(√n) support function\n");
    printf("  - Automatic AABB calculation for polyhedra\n");
    printf("  - Seamless integration with existing collision system\n");
    printf("  - Stacked cubes use BoxCollider for stable stacking\n");
    
    return 0;
}