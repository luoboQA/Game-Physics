#include "RigidBody.h"
#include "Collider.h"
#include "PolyhedronCollider.h"
#include "PhysicsWorld.h"
#include "Broadphase.h"
#include "ContactConstraint.h"
#include "OpenGLRenderer.h"
#include <vector>
#include <cstdio>

PhysicsWorld world(Vec3(0, -9.8f, 0));
std::vector<RigidBody*> bodies;
OpenGLRenderer renderer;

RigidBody* CreatePolyhedronCube(const Vec3& pos, const Vec3& halfExtents, 
                                 float mass, float r, float g, float b) {
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
    
    std::vector<int> indices = {
        0,1,2, 0,2,3, 4,6,5, 4,7,6,
        0,5,1, 0,4,5, 3,2,6, 3,6,7,
        0,3,7, 0,7,4, 1,5,6, 1,6,2
    };
    
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
    body->UpdateOrientationAndInertia();
    body->UpdateGlobalCentroidFromPosition();
    
    PolyhedronCollider* collider = new PolyhedronCollider(body, Vec3(0,0,0), vertices, indices);
    body->AddCollider(collider);
    world.AddRigidBody(body);
    bodies.push_back(body);
    renderer.AddObject(body, collider, r, g, b);
    
    return body;
}

RigidBody* CreateSimpleBox(const Vec3& pos, const Vec3& halfExtents, float mass,
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

RigidBody* CreateTetrahedron(const Vec3& pos, float size, float mass,
                              float r, float g, float b) {
    float h = size * sqrtf(6.0f) / 4.0f;
    
    std::vector<Vec3> vertices = {
        Vec3(0, 0, 0),
        Vec3(size, 0, 0),
        Vec3(size/2, 0, size * sqrtf(3.0f)/2),
        Vec3(size/2, h, size * sqrtf(3.0f)/6)
    };
    
    std::vector<int> indices = {
        0,1,2, 0,3,1, 1,3,2, 2,3,0
    };
    
    float approxInertia = 0.1f * mass * size * size;
    Mat3 inertia;
    inertia.m[0][0] = approxInertia;
    inertia.m[1][1] = approxInertia;
    inertia.m[2][2] = approxInertia;
    
    RigidBody* body = new RigidBody(mass, inertia);
    body->m_position = pos;
    body->UpdateOrientationAndInertia();
    body->UpdateGlobalCentroidFromPosition();
    
    Vec3 centroid(0,0,0);
    for (const auto& v : vertices) centroid = centroid + v;
    centroid = centroid * (1.0f / vertices.size());
    
    std::vector<Vec3> centeredVertices;
    for (const auto& v : vertices) centeredVertices.push_back(v - centroid);
    
    PolyhedronCollider* collider = new PolyhedronCollider(body, centroid, centeredVertices, indices);
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
        printf("--- Frame %d (Polyhedron Demo) ---\n", frame);
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
    printf("Stage 6: Polyhedron Collider (Visual)\n");
    printf("GJK + EPA with Arbitrary Convex Shapes\n");
    printf("========================================\n\n");
    
    world.SetNumIterations(20);
    world.SetDefaultRestitution(0.3f);
    world.SetDefaultFriction(0.5f);
    world.EnableFriction(true);
    world.EnableWarmStarting(true);
    world.SetPenetrationSlop(0.005f);
    world.SetRestitutionSlop(0.3f);
    
    // 地面
    CreateSimpleBox(Vec3(0, -5, 0), Vec3(15, 0.5f, 15), 0.0f, 0.4f, 0.4f, 0.4f, true);
    printf("Ground: top at y=%.2f\n\n", -5.0f + 0.5f);
    
    // 多面体立方体（红色）
    CreatePolyhedronCube(Vec3(0, 2.5f, 0), Vec3(0.5f, 0.5f, 0.5f), 1.0f, 1.0f, 0.3f, 0.3f);
    
    // 球体（蓝色）
    CreateSphere(Vec3(1.5f, 2.5f, 0), 0.5f, 1.0f, 0.3f, 0.5f, 1.0f);
    
    // 四面体（紫色）
    CreateTetrahedron(Vec3(-1.5f, 2.5f, 0), 0.8f, 1.0f, 0.8f, 0.3f, 1.0f);
    
    // 堆叠测试 - 使用简单盒子
    printf("\n[Note] Stacked cubes use BoxCollider for stable stacking\n");
    CreateSimpleBox(Vec3(2.0f, 0.6f, 1.5f), Vec3(0.4f, 0.4f, 0.4f), 1.0f, 0.3f, 1.0f, 0.3f);
    CreateSimpleBox(Vec3(2.0f, 1.4f, 1.5f), Vec3(0.4f, 0.4f, 0.4f), 1.0f, 1.0f, 1.0f, 0.3f);
    
    printf("\nObjects:\n");
    printf("  - Cube (polyhedron, 8 vertices, 12 faces) - RED\n");
    printf("  - Sphere (implicit shape) - BLUE\n");
    printf("  - Tetrahedron (polyhedron, 4 vertices, 4 faces) - PURPLE\n");
    printf("  - Stacked cubes (BoxCollider) - GREEN/YELLOW\n");
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