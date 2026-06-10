#ifndef GJK_H
#define GJK_H

#include "Vec3.h"
#include "Collider.h"
#include <vector>

// CSO支撑点结构
struct CSOSupport {
    Vec3 csoPoint;      // CSO上的点 (supportA - supportB)
    Vec3 pointA;        // 物体A上的世界空间支撑点
    Vec3 pointB;        // 物体B上的世界空间支撑点
    Vec3 localPointA;   // 物体A上的局部空间支撑点
    Vec3 localPointB;   // 物体B上的局部空间支撑点
};

// GJK结果
struct GJKResult {
    std::vector<CSOSupport> simplex;  // 最终单纯形（包含原点）
    bool isColliding;
};

// 辅助函数：计算CSO支撑点
CSOSupport GetCSOSupport(const Collider* colliderA, const Collider* colliderB, const Vec3& direction);

// 单纯形操作函数
bool UpdateSimplex(std::vector<CSOSupport>& simplex, Vec3& newDirection);
bool LineCase(const std::vector<CSOSupport>& simplex, Vec3& newDirection);
bool TriangleCase(std::vector<CSOSupport>& simplex, Vec3& newDirection);
bool TetrahedronCase(std::vector<CSOSupport>& simplex, Vec3& newDirection);

// GJK主算法
bool GJK(const Collider* colliderA, const Collider* colliderB, GJKResult* result = nullptr);

#endif