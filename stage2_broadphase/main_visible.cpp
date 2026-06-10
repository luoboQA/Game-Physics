#include "RigidBody.h"
#include "Collider.h"
#include "PhysicsWorld.h"
#include "Broadphase.h"
#include "OpenGLRenderer.h"
#include <vector>

PhysicsWorld world(Vec3(0, -9.8f, 0));
std::vector<RigidBody*> bodies;
std::vector<BoxCollider*> boxColliders;
OpenGLRenderer renderer;
bool running = true;

void UpdatePhysics() {
    if (!running) return;
    world.Step(1.0f / 60.0f);
    
    // 检查是否有物体掉出视野，重置位置
    static int frame = 0;
    if (++frame % 300 == 0) {
        for (auto body : bodies) {
            if (!body->m_isStatic && body->m_position.y < -8.0f) {
                // 重置到顶部
                body->m_position.y = 8.0f;
                body->m_linearVelocity = Vec3(0, 0, 0);
                body->UpdateGlobalCentroidFromPosition();
                printf("Reset cube to top\n");
            }
        }
    }
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

int main(int argc, char** argv) {
    printf("=== Stage 2: Broadphase + AABB (Visual) ===\n");
    printf("Falling cubes - they reset to top after hitting ground\n");
    printf("Press ESC or 'q' to exit\n\n");
    
    // 地面
    CreateBox(Vec3(0, -5, 0), Vec3(15, 0.5f, 15), 0.0f, 0.3f, 0.5f, 0.2f, true);
    
    // 下落的立方体 - 从高处开始
    for (int i = 0; i < 5; i++) {
        float hue = (float)i / 5.0f;
        CreateBox(Vec3(i * 1.5f - 3.0f, 6.0f + i * 0.5f, 0), Vec3(0.5f, 0.5f, 0.5f), 1.0f, 
                  hue, 0.5f, 1.0f - hue);
    }
    
    printf("Objects: 5 falling cubes (reset to top when below -8)\n");
    
    renderer.Init(argc, argv, 800, 600);
    
    glutTimerFunc(16, [](int) {
        UpdatePhysics();
        glutPostRedisplay();
        glutTimerFunc(16, [](int) {}, 0);
    }, 0);
    
    renderer.Run(1.0f/60.0f);
    
    return 0;
}