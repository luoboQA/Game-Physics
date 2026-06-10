#include "GJK.h"
#include "RigidBody.h"
#include <cstdio>
#include <algorithm>
#include <cmath>

const float GJK_EPSILON = 1e-6f;
const float GJK_EPSILON_SQ = GJK_EPSILON * GJK_EPSILON;

CSOSupport GetCSOSupport(const Collider* colliderA, const Collider* colliderB, const Vec3& direction) {
    CSOSupport result;
    
    // 确保方向向量是单位向量
    Vec3 dirNorm = direction;
    float len = dirNorm.Length();
    if (len > GJK_EPSILON) {
        dirNorm = dirNorm * (1.0f / len);
    } else {
        dirNorm = Vec3(1, 0, 0);
    }
    
    // 获取世界空间的支撑点
    result.pointA = colliderA->SupportWorld(dirNorm);
    result.pointB = colliderB->SupportWorld(-dirNorm);
    
    // 存储局部空间的支撑点（用于EPA）
    Vec3 localDirA = colliderA->body ? colliderA->body->GlobalToLocalVec(dirNorm) : dirNorm;
    Vec3 localDirB = colliderB->body ? colliderB->body->GlobalToLocalVec(-dirNorm) : -dirNorm;
    
    result.localPointA = colliderA->SupportLocal(localDirA);
    result.localPointB = colliderB->SupportLocal(localDirB);
    
    // CSO支撑点 = A点 - B点
    result.csoPoint = result.pointA - result.pointB;
    
    return result;
}

// 处理线段情况
bool LineCase(const std::vector<CSOSupport>& simplex, Vec3& newDirection) {
    const CSOSupport& A = simplex[0];
    const CSOSupport& B = simplex[1];
    
    Vec3 AB = B.csoPoint - A.csoPoint;
    Vec3 AO = -A.csoPoint;
    
    float ab_len_sq = AB.Dot(AB);
    if (ab_len_sq < GJK_EPSILON_SQ) {
        newDirection = AO;
        return false;
    }
    
    float t = AB.Dot(AO) / ab_len_sq;
    
    if (t <= 0.0f) {
        newDirection = AO;
        return false;
    } else if (t >= 1.0f) {
        newDirection = -B.csoPoint;
        return false;
    } else {
        Vec3 closestPoint = A.csoPoint + AB * t;
        newDirection = -closestPoint;
        float distSq = newDirection.LengthSq();
        return distSq < GJK_EPSILON_SQ;
    }
}

// 处理三角形情况
bool TriangleCase(std::vector<CSOSupport>& simplex, Vec3& newDirection) {
    const CSOSupport& A = simplex[0];
    const CSOSupport& B = simplex[1];
    const CSOSupport& C = simplex[2];
    
    Vec3 AB = B.csoPoint - A.csoPoint;
    Vec3 AC = C.csoPoint - A.csoPoint;
    Vec3 AO = -A.csoPoint;
    
    Vec3 normal = AB.Cross(AC);
    float normal_len = normal.Length();
    if (normal_len < GJK_EPSILON) {
        simplex.erase(simplex.begin() + 2);
        return LineCase(simplex, newDirection);
    }
    normal = normal * (1.0f / normal_len);
    
    float distToPlane = normal.Dot(AO);
    
    if (fabs(distToPlane) < GJK_EPSILON) {
        newDirection = normal;
        if (newDirection.Dot(AO) < 0) {
            newDirection = -newDirection;
        }
        return true;
    }
    
    if (distToPlane < 0) {
        normal = -normal;
        distToPlane = -distToPlane;
    }
    
    Vec3 perpAB = normal.Cross(AB);
    Vec3 perpAC = AC.Cross(normal);
    
    bool regionAB = perpAB.Dot(AO) > 0;
    bool regionAC = perpAC.Dot(AO) > 0;
    
    if (regionAB && regionAC) {
        newDirection = normal;
        return false;
    }
    
    if (regionAB) {
        simplex.erase(simplex.begin() + 2);
        return LineCase(simplex, newDirection);
    }
    
    if (regionAC) {
        simplex.erase(simplex.begin() + 1);
        return LineCase(simplex, newDirection);
    }
    
    simplex.erase(simplex.begin() + 1, simplex.begin() + 3);
    newDirection = AO;
    return false;
}

// 处理四面体情况
bool TetrahedronCase(std::vector<CSOSupport>& simplex, Vec3& newDirection) {
    const CSOSupport& A = simplex[0];
    const CSOSupport& B = simplex[1];
    const CSOSupport& C = simplex[2];
    const CSOSupport& D = simplex[3];
    
    Vec3 AB = B.csoPoint - A.csoPoint;
    Vec3 AC = C.csoPoint - A.csoPoint;
    Vec3 AD = D.csoPoint - A.csoPoint;
    Vec3 AO = -A.csoPoint;
    
    Vec3 normalABC = AB.Cross(AC);
    Vec3 normalABD = AB.Cross(AD);
    Vec3 normalACD = AC.Cross(AD);
    Vec3 normalBCD = (C.csoPoint - B.csoPoint).Cross(D.csoPoint - B.csoPoint);
    
    auto normalizeAndOrient = [&](Vec3& normal, const Vec3& toOpposite) {
        float len = normal.Length();
        if (len < GJK_EPSILON) return false;
        normal = normal * (1.0f / len);
        if (normal.Dot(toOpposite) > 0) {
            normal = -normal;
        }
        return true;
    };
    
    if (!normalizeAndOrient(normalABC, AD)) return false;
    if (!normalizeAndOrient(normalABD, AC)) return false;
    if (!normalizeAndOrient(normalACD, AB)) return false;
    
    Vec3 BC = C.csoPoint - B.csoPoint;
    Vec3 BD = D.csoPoint - B.csoPoint;
    Vec3 BO = -B.csoPoint;
    normalBCD = BC.Cross(BD);
    if (!normalizeAndOrient(normalBCD, A.csoPoint - B.csoPoint)) return false;
    
    if (normalABC.Dot(AO) > 0) {
        simplex.pop_back();
        return TriangleCase(simplex, newDirection);
    }
    
    if (normalABD.Dot(AO) > 0) {
        simplex.erase(simplex.begin() + 2);
        return TriangleCase(simplex, newDirection);
    }
    
    if (normalACD.Dot(AO) > 0) {
        simplex.erase(simplex.begin() + 1);
        return TriangleCase(simplex, newDirection);
    }
    
    if (normalBCD.Dot(BO) > 0) {
        simplex.erase(simplex.begin());
        return TriangleCase(simplex, newDirection);
    }
    
    newDirection = Vec3(0, 0, 0);
    return true;
}

// 更新单纯形
bool UpdateSimplex(std::vector<CSOSupport>& simplex, Vec3& newDirection) {
    switch (simplex.size()) {
        case 1:
            newDirection = -simplex[0].csoPoint;
            return false;
        case 2:
            return LineCase(simplex, newDirection);
        case 3:
            return TriangleCase(simplex, newDirection);
        case 4:
            return TetrahedronCase(simplex, newDirection);
        default:
            return false;
    }
}

// GJK主算法
bool GJK(const Collider* colliderA, const Collider* colliderB, GJKResult* result) {
    if (!colliderA || !colliderB) return false;
    
    std::vector<CSOSupport> simplex;
    simplex.reserve(4);
    
    // 使用从A质心指向B质心的方向作为初始搜索方向
    Vec3 centerA = colliderA->GetWorldCentroid();
    Vec3 centerB = colliderB->GetWorldCentroid();
    Vec3 searchDir = centerB - centerA;
    
    if (searchDir.LengthSq() < GJK_EPSILON_SQ) {
        searchDir = Vec3(1.0f, 0.0f, 0.0f);
    } else {
        searchDir = searchDir.Normalized();
    }
    
    // 获取第一个支撑点
    CSOSupport support = GetCSOSupport(colliderA, colliderB, searchDir);
    simplex.push_back(support);
    
    searchDir = -support.csoPoint;
    if (searchDir.LengthSq() < GJK_EPSILON_SQ) {
        searchDir = Vec3(1.0f, 0.0f, 0.0f);
    } else {
        searchDir = searchDir.Normalized();
    }
    
    int maxIterations = 100;
    int iteration = 0;
    
    while (iteration++ < maxIterations) {
        CSOSupport newSupport = GetCSOSupport(colliderA, colliderB, searchDir);
        
        if (newSupport.csoPoint.Dot(searchDir) <= 0.0f) {
            if (result) {
                result->isColliding = false;
                result->simplex = simplex;
            }
            return false;
        }
        
        simplex.push_back(newSupport);
        
        if (UpdateSimplex(simplex, searchDir)) {
            if (result) {
                result->isColliding = true;
                result->simplex = simplex;
            }
            return true;
        }
        
        if (searchDir.LengthSq() < GJK_EPSILON_SQ) {
            if (result) {
                result->isColliding = true;
                result->simplex = simplex;
            }
            return true;
        }
        
        searchDir = searchDir.Normalized();
    }
    
    if (result) {
        result->isColliding = false;
    }
    return false;
}