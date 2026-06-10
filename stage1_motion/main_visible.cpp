#include "RigidBody.h"
#include "Collider.h"
#include "OpenGLRenderer.h"
#include <cstdio>

RigidBody* cube;
BoxCollider* collider;
Vec3 force(0, 10, 0);
Vec3 applicationPoint(1, 0, 0);
float time = 0;

void UpdatePhysics() {
    static float lastTime = 0;
    float dt = 0.016f;  // ~60 FPS
    
    // 持续施加力
    cube->ApplyForce(force, applicationPoint);
    cube->Integrate(dt);
    
    time += dt;
    
    // 每60帧打印一次状态
    static int frame = 0;
    if (++frame % 60 == 0) {
        printf("Time: %.2f, Pos: (%.2f, %.2f, %.2f), Vel: (%.2f, %.2f, %.2f)\n",
               time, cube->m_position.x, cube->m_position.y, cube->m_position.z,
               cube->m_linearVelocity.x, cube->m_linearVelocity.y, cube->m_linearVelocity.z);
    }
}

int main(int argc, char** argv) {
    printf("=== Stage 1: Motion Dynamics (Visual) ===\n");
    printf("Force (0,10,0) applied at (1,0,0) - cube moves up and rotates\n");
    printf("Press ESC or 'q' to exit\n\n");
    
    // 创建刚体 - 从 y=0 开始
    Mat3 inertia = Mat3::Identity();
    cube = new RigidBody(1.0f, inertia);
    cube->m_position = Vec3(0, 0, 0);
    cube->m_orientation = Mat3::Identity();
    cube->UpdateOrientationAndInertia();
    cube->UpdateGlobalCentroidFromPosition();
    
    // 添加碰撞体用于可视化
    collider = new BoxCollider(cube, Vec3(0,0,0), Vec3(0.5f, 0.5f, 0.5f));
    cube->AddCollider(collider);
    
    // 初始化渲染器
    OpenGLRenderer renderer;
    renderer.Init(argc, argv, 800, 600);
    renderer.AddObject(cube, collider, 1.0f, 0.5f, 0.2f);
    
    glutTimerFunc(16, [](int) {
        UpdatePhysics();
        glutPostRedisplay();
        glutTimerFunc(16, [](int) {}, 0);
    }, 0);
    
    renderer.Run(1.0f/60.0f);
    
    delete cube;
    return 0;
}