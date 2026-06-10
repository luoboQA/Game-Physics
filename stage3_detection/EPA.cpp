#include "EPA.h"
#include "RigidBody.h"
#include <cfloat>
#include <cmath>
#include <algorithm>

bool EPA(const Collider* colliderA, const Collider* colliderB,
         const std::vector<CSOSupport>& initialSimplex,
         ContactManifold* result) {
    
    if (!result || initialSimplex.size() < 2) return false;
    
    // 找到CSO中离原点最近的点
    Vec3 closestPoint(0, 0, 0);
    Vec3 closestLocalA(0, 0, 0);
    Vec3 closestLocalB(0, 0, 0);
    float minDistSq = FLT_MAX;
    
    // 检查所有单纯形顶点
    for (const auto& support : initialSimplex) {
        float distSq = support.csoPoint.LengthSq();
        if (distSq < minDistSq) {
            minDistSq = distSq;
            closestPoint = support.csoPoint;
            closestLocalA = support.localPointA;
            closestLocalB = support.localPointB;
        }
    }
    
    // 检查边上的最近点
    if (initialSimplex.size() >= 2) {
        for (size_t i = 0; i < initialSimplex.size(); ++i) {
            for (size_t j = i + 1; j < initialSimplex.size(); ++j) {
                const Vec3& p1 = initialSimplex[i].csoPoint;
                const Vec3& p2 = initialSimplex[j].csoPoint;
                Vec3 edge = p2 - p1;
                float edgeLenSq = edge.Dot(edge);
                
                if (edgeLenSq > 1e-8f) {
                    float t = -p1.Dot(edge) / edgeLenSq;
                    if (t > 0 && t < 1) {
                        Vec3 pointOnEdge = p1 + edge * t;
                        float distSq = pointOnEdge.LengthSq();
                        if (distSq < minDistSq) {
                            minDistSq = distSq;
                            closestPoint = pointOnEdge;
                            float u = 1.0f - t;
                            float v = t;
                            closestLocalA = initialSimplex[i].localPointA * u + initialSimplex[j].localPointA * v;
                            closestLocalB = initialSimplex[i].localPointB * u + initialSimplex[j].localPointB * v;
                        }
                    }
                }
            }
        }
    }
    
    // 检查三角形面上的最近点
    if (initialSimplex.size() >= 3) {
        for (size_t i = 0; i < initialSimplex.size(); ++i) {
            for (size_t j = i + 1; j < initialSimplex.size(); ++j) {
                for (size_t k = j + 1; k < initialSimplex.size(); ++k) {
                    const Vec3& p1 = initialSimplex[i].csoPoint;
                    const Vec3& p2 = initialSimplex[j].csoPoint;
                    const Vec3& p3 = initialSimplex[k].csoPoint;
                    
                    float u, v, w;
                    Vec3 closestOnTri = ClosestPointOnTriangle(Vec3(0,0,0), p1, p2, p3, u, v, w);
                    float distSq = closestOnTri.LengthSq();
                    
                    if (distSq < minDistSq) {
                        minDistSq = distSq;
                        closestPoint = closestOnTri;
                        closestLocalA = initialSimplex[i].localPointA * u + 
                                       initialSimplex[j].localPointA * v + 
                                       initialSimplex[k].localPointA * w;
                        closestLocalB = initialSimplex[i].localPointB * u + 
                                       initialSimplex[j].localPointB * v + 
                                       initialSimplex[k].localPointB * w;
                    }
                }
            }
        }
    }
    
    float depth = sqrtf(minDistSq);
    if (depth < 1e-6f) return false;
    
    // 法线：从 closestPoint 指向原点
    result->normal = closestPoint;
    result->normal.Negate();
    
    float len = result->normal.Length();
    if (len > 1e-6f) {
        result->normal = result->normal * (1.0f / len);
    } else {
        result->normal = Vec3(0, 1, 0);
    }
    
    // 确保法线从静态物体指向动态物体
    if (colliderB->body && colliderB->body->m_isStatic && result->normal.y < 0) {
        result->normal = -result->normal;
    }
    if (colliderA->body && colliderA->body->m_isStatic && result->normal.y > 0) {
        result->normal = -result->normal;
    }
    
    result->penetrationDepth = depth;
    result->localContactA = closestLocalA;
    result->localContactB = closestLocalB;
    
    if (colliderA->body) {
        result->contactPointA = colliderA->body->LocalToGlobal(result->localContactA);
    } else {
        result->contactPointA = result->localContactA;
    }
    
    if (colliderB->body) {
        result->contactPointB = colliderB->body->LocalToGlobal(result->localContactB);
    } else {
        result->contactPointB = result->localContactB;
    }
    
    return true;
}

// ClosestPointOnTriangle 函数
Vec3 ClosestPointOnTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c, 
                            float& u, float& v, float& w) {
    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 ap = p - a;
    
    float d1 = ab.Dot(ap);
    float d2 = ac.Dot(ap);
    
    if (d1 <= 0.0f && d2 <= 0.0f) {
        u = 1.0f; v = 0.0f; w = 0.0f;
        return a;
    }
    
    Vec3 bp = p - b;
    float d3 = ab.Dot(bp);
    float d4 = ac.Dot(bp);
    
    if (d3 >= 0.0f && d4 <= d3) {
        u = 0.0f; v = 1.0f; w = 0.0f;
        return b;
    }
    
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        v = d1 / (d1 - d3);
        u = 1.0f - v;
        w = 0.0f;
        return a + ab * v;
    }
    
    Vec3 cp = p - c;
    float d5 = ab.Dot(cp);
    float d6 = ac.Dot(cp);
    
    if (d6 >= 0.0f && d5 <= d6) {
        u = 0.0f; v = 0.0f; w = 1.0f;
        return c;
    }
    
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        w = d2 / (d2 - d6);
        u = 1.0f - w;
        v = 0.0f;
        return a + ac * w;
    }
    
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        u = 0.0f;
        v = 1.0f - w;
        return b + (c - b) * w;
    }
    
    float denom = 1.0f / (va + vb + vc);
    v = vb * denom;
    w = vc * denom;
    u = 1.0f - v - w;
    return a + ab * v + ac * w;
}