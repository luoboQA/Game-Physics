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
        printf("--- Frame %d ---\n", frame);
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
    printf("Stage 4: Collision Resolution (Visual)\n");
    printf("Sequential Impulse + Friction\n");
    printf("========================================\n\n");
    
    world.SetNumIterations(10);
    world.SetDefaultRestitution(0.5f);
    world.SetDefaultFriction(0.5f);
    world.EnableFriction(true);
    
    // 地面
    CreateBox(Vec3(0, -5, 0), Vec3(10, 0.5f, 10), 0.0f, 0.3f, 0.5f, 0.2f, true);
    printf("Ground: top at y=%.2f\n\n", -5.0f + 0.5f);
    
    // 立方体（红色）
    CreateBox(Vec3(0, 3, 0), Vec3(0.5f, 0.5f, 0.5f), 1.0f, 1.0f, 0.3f, 0.3f);
    
    // 球体（蓝色）
    CreateSphere(Vec3(1.5f, 3, 0), 0.5f, 1.0f, 0.3f, 0.5f, 1.0f);
    
    // 堆叠测试 - 底部盒子（绿色）
    CreateBox(Vec3(-1.5f, 0.6f, 0), Vec3(0.4f, 0.4f, 0.4f), 1.0f, 0.3f, 1.0f, 0.3f);
    
    // 堆叠测试 - 顶部盒子（黄色）
    CreateBox(Vec3(-1.5f, 1.4f, 0), Vec3(0.4f, 0.4f, 0.4f), 1.0f, 1.0f, 1.0f, 0.3f);
    
    printf("Objects: Cube(red), Sphere(blue), Stacked boxes(green,yellow)\n");
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