#include "RigidBody.h"
#include "Collider.h"
#include "PhysicsWorld.h"
#include "Broadphase.h"
#include "ContactConstraint.h"
#include "OpenGLRenderer.h"
#include <vector>
#include <cstdio>

PhysicsWorld world(Vec3(0, -9.8f, 0));
std::vector<RigidBody*> bodies;
OpenGLRenderer renderer;

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
    world.Step(1.0f / 60.0f);
    
    if (++frame % 60 == 0 && !bodies.empty()) {
        printf("--- Frame %d (Stability Optimized) ---\n", frame);
        for (size_t i = 0; i < bodies.size(); i++) {
            if (!bodies[i]->m_isStatic) {
                printf("  Body %zu: y=%.2f, vy=%.2f\n", 
                       i, bodies[i]->m_position.y, bodies[i]->m_linearVelocity.y);
            }
        }
    }
}

int main(int argc, char** argv) {
    printf("========================================\n");
    printf("Stage 5: Stability Optimizations (Visual)\n");
    printf("Warm Starting + Slop Parameters\n");
    printf("========================================\n\n");
    
    // 配置稳定性参数
    world.SetNumIterations(12);
    world.SetDefaultRestitution(0.3f);
    world.SetDefaultFriction(0.5f);
    world.EnableFriction(true);
    world.EnableWarmStarting(true);
    world.SetWarmStartFactor(0.7f);
    world.SetPenetrationSlop(0.005f);
    world.SetRestitutionSlop(0.3f);
    
    printf("Stability Settings:\n");
    printf("  - Iterations: 12\n");
    printf("  - Warm Starting: ON (factor: 0.7)\n");
    printf("  - Penetration Slop: 0.005\n");
    printf("  - Restitution Slop: 0.3\n\n");
    
    // 地面（灰色）
    CreateBox(Vec3(0, -5, 0), Vec3(15, 0.5f, 15), 0.0f, 0.4f, 0.4f, 0.4f, true);
    printf("Ground: top at y=%.2f\n\n", -5.0f + 0.5f);
    
    // 立方体（红色）
    CreateBox(Vec3(0, 2.0f, 0), Vec3(0.5f, 0.5f, 0.5f), 1.0f, 1.0f, 0.3f, 0.3f);
    
    // 球体（蓝色）
    CreateSphere(Vec3(1.5f, 1.8f, 0), 0.5f, 1.0f, 0.3f, 0.5f, 1.0f);
    
    // 堆叠测试 - 3层堆叠（绿色、黄色、橙色）
    CreateBox(Vec3(-1.5f, 0.6f, 1.5f), Vec3(0.4f, 0.4f, 0.4f), 1.0f, 0.3f, 1.0f, 0.3f);
    CreateBox(Vec3(-1.5f, 1.4f, 1.5f), Vec3(0.4f, 0.4f, 0.4f), 1.0f, 1.0f, 1.0f, 0.3f);
    CreateBox(Vec3(-1.5f, 2.2f, 1.5f), Vec3(0.4f, 0.4f, 0.4f), 1.0f, 1.0f, 0.5f, 0.3f);
    
    printf("Objects: Cube(red), Sphere(blue), 3-layer Stack(green,yellow,orange)\n");
    printf("Press ESC or 'q' to exit\n\n");
    
    renderer.Init(argc, argv, 800, 600);
    
    glutTimerFunc(16, [](int) {
        UpdatePhysics();
        glutPostRedisplay();
        glutTimerFunc(16, [](int) {}, 0);
    }, 0);
    
    renderer.Run(1.0f/60.0f);
    
    return 0;
}