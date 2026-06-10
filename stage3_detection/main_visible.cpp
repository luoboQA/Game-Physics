#include "RigidBody.h"
#include "Collider.h"
#include "PhysicsWorld.h"
#include "Broadphase.h"
#include "GJK.h"
#include "EPA.h"
#include "OpenGLRenderer.h"
#include <vector>

PhysicsWorld world(Vec3(0, -9.8f, 0));
std::vector<RigidBody*> bodies;
OpenGLRenderer renderer;

void ResetPhysics() {
    // 重置所有物体的位置
    for (auto body : bodies) {
        if (!body->m_isStatic) {
            body->m_position.y = 6.0f;
            body->m_linearVelocity = Vec3(0, 0, 0);
            body->m_angularVelocity = Vec3(0, 0, 0);
            body->UpdateGlobalCentroidFromPosition();
        }
    }
    printf("=== Physics Reset ===\n");
}

RigidBody* CreateBox(const Vec3& pos, const Vec3& halfExtents, float mass, 
                      float r, float g, float b, bool isStatic = false) {
    float w = halfExtents.x * 2;
    float h = halfExtents.y * 2;
    float d = halfExtents.z * 2;
    
    float ix = (1.0f/12.0f) * mass * (h*h + d*d);
    float iy = (1.0f/12.0f) * mass * (w*w + d*d);
    float iz = (1.0f/12.0f) * mass * (w*w + h*h);
    
    Mat3 inertia;
    inertia.m[0][0] = ix;
    inertia.m[1][1] = iy;
    inertia.m[2][2] = iz;
    
    RigidBody* body = new RigidBody(mass, inertia);
    body->m_position = pos;
    if (isStatic) body->m_isStatic = true;
    body->UpdateOrientationAndInertia();
    body->UpdateGlobalCentroidFromPosition();
    
    BoxCollider* collider = new BoxCollider(body, Vec3(0,0,0), halfExtents);
    body->AddCollider(collider);
    world.AddRigidBody(body);
    bodies.push_back(body);
    renderer.AddObject(body, collider, r, g, b);
    
    return body;
}

RigidBody* CreateSphere(const Vec3& pos, float radius, float mass,
                         float r, float g, float b) {
    float inertiaVal = (2.0f/5.0f) * mass * radius * radius;
    Mat3 inertia;
    inertia.m[0][0] = inertiaVal;
    inertia.m[1][1] = inertiaVal;
    inertia.m[2][2] = inertiaVal;
    
    RigidBody* body = new RigidBody(mass, inertia);
    body->m_position = pos;
    body->UpdateOrientationAndInertia();
    body->UpdateGlobalCentroidFromPosition();
    
    SphereCollider* collider = new SphereCollider(body, Vec3(0,0,0), radius);
    body->AddCollider(collider);
    world.AddRigidBody(body);
    bodies.push_back(body);
    renderer.AddObject(body, collider, r, g, b);
    
    return body;
}

void UpdatePhysics() {
    static int frame = 0;
    static int resetCounter = 0;
    world.Step(1.0f / 60.0f);
    
    if (++frame % 60 == 0) {
        // 检查是否需要重置
        bool needsReset = false;
        for (auto body : bodies) {
            if (!body->m_isStatic && body->m_position.y < -6.0f) {
                needsReset = true;
                break;
            }
        }
        
        if (needsReset) {
            resetCounter++;
            if (resetCounter >= 3) {
                ResetPhysics();
                resetCounter = 0;
            }
        }
        
        // 打印碰撞信息
        const auto& manifolds = world.GetContactManifolds();
        if (!manifolds.empty()) {
            printf("Collision! Depth: %.4f\n", manifolds[0].penetrationDepth);
        }
    }
}

int main(int argc, char** argv) {
    printf("=== Stage 3: GJK + EPA Collision Detection (Visual) ===\n");
    printf("Cube and sphere falling - resets after hitting ground\n");
    printf("Press ESC or 'q' to exit\n\n");
    
    // 地面
    CreateBox(Vec3(0, -5, 0), Vec3(10, 0.5f, 10), 0.0f, 0.3f, 0.5f, 0.2f, true);
    
    // 立方体（红色）
    CreateBox(Vec3(-1, 6, 0), Vec3(0.5f, 0.5f, 0.5f), 1.0f, 1.0f, 0.3f, 0.3f);
    
    // 球体（蓝色）
    CreateSphere(Vec3(1.5, 6, 0), 0.5f, 1.0f, 0.3f, 0.5f, 1.0f);
    
    printf("Objects: Cube(red), Sphere(blue) - will reset after 3 impacts\n");
    
    renderer.Init(argc, argv, 800, 600);
    
    glutTimerFunc(16, [](int) {
        UpdatePhysics();
        glutPostRedisplay();
        glutTimerFunc(16, [](int) {}, 0);
    }, 0);
    
    renderer.Run(1.0f/60.0f);
    
    return 0;
}