#ifndef EPA_H
#define EPA_H

#include "Vec3.h"
#include "Collider.h"
#include "GJK.h"
#include <vector>

// 接触面（三角形）
struct EPATriangle {
    CSOSupport vertices[3];
    Vec3 normal;
    float distance;
};

// 接触信息
struct ContactManifold {
    Vec3 contactPointA;      // 物体A上的接触点（世界空间）
    Vec3 contactPointB;      // 物体B上的接触点（世界空间）
    Vec3 localContactA;      // 物体A上的接触点（局部空间）
    Vec3 localContactB;      // 物体B上的接触点（局部空间）
    Vec3 normal;             // 接触法线（从A指向B）
    float penetrationDepth;  // 穿透深度
};

// EPA算法
bool EPA(const Collider* colliderA, const Collider* colliderB, 
         const std::vector<CSOSupport>& initialSimplex,
         ContactManifold* result);

// 辅助函数
Vec3 ClosestPointOnTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c, 
                            float& u, float& v, float& w);

#endif