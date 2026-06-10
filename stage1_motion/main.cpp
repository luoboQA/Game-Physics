#include "RigidBody.h"
#include <cstdio>

int main() {
    printf("=== Stage 1: Motion Dynamics ===\n\n");
    
    // 创建立方体刚体: 质量1.0，惯性张量为单位阵
    Mat3 inertia = Mat3::Identity();
    RigidBody cube(1.0f, inertia);
    
    printf("Initial state:\n");
    cube.Print();
    printf("\n");
    
    printf("Applying constant force (0,10,0) at (1,0,0) for 1 second\n");
    printf("Simulating 1 second at 60 FPS...\n\n");
    
    // 模拟 60fps 下的 1 秒
    float dt = 1.0f / 60.0f;
    Vec3 force(0, 10, 0);
    Vec3 applicationPoint(1, 0, 0);
    
    for (int step = 0; step < 60; ++step) {
        // 每一帧都施加力（持续力）
        cube.ApplyForce(force, applicationPoint);
        cube.Integrate(dt);
    }
    
    printf("Final state after 1 second:\n");
    cube.Print();
    
    return 0;
}