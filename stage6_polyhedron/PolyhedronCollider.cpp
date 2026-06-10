#include "PolyhedronCollider.h"
#include "RigidBody.h"
#include <cstdio>
#include <algorithm>
#include <cmath>
#include <cfloat>

PolyhedronCollider::PolyhedronCollider(RigidBody* b, const Vec3& lc,
                                       const std::vector<Vec3>& vertices,
                                       const std::vector<int>& indices)
    : Collider(b, COLLIDER_POLYHEDRON, lc, AABB()),
      m_useBruteForceAABB(vertices.size() < 32) {
    
    BuildFromTriangles(vertices, indices);
    
    // 更新本地AABB
    if (!vertices.empty()) {
        Vec3 minPoint = vertices[0];
        Vec3 maxPoint = vertices[0];
        for (const auto& v : vertices) {
            minPoint.x = std::min(minPoint.x, v.x);
            minPoint.y = std::min(minPoint.y, v.y);
            minPoint.z = std::min(minPoint.z, v.z);
            maxPoint.x = std::max(maxPoint.x, v.x);
            maxPoint.y = std::max(maxPoint.y, v.y);
            maxPoint.z = std::max(maxPoint.z, v.z);
        }
        localAABB = AABB(minPoint + lc, maxPoint + lc);
    }
    
    // 缓存本地顶点（转换到局部质心坐标系）
    m_localVertices = vertices;
    for (auto& v : m_localVertices) {
        v = v + lc;
    }
    
    printf("[PolyhedronCollider] Created: lc=(%.2f,%.2f,%.2f), %zu vertices, %u faces\n",
           lc.x, lc.y, lc.z, vertices.size(), mesh.numFaces);
}

PolyhedronCollider::PolyhedronCollider(RigidBody* b, const Vec3& lc,
                                       const std::vector<Vec3>& vertices)
    : Collider(b, COLLIDER_POLYHEDRON, lc, AABB()),
      m_useBruteForceAABB(vertices.size() < 32) {
    
    TriangulateConvex(vertices);
    
    // 更新本地AABB
    if (!vertices.empty()) {
        Vec3 minPoint = vertices[0];
        Vec3 maxPoint = vertices[0];
        for (const auto& v : vertices) {
            minPoint.x = std::min(minPoint.x, v.x);
            minPoint.y = std::min(minPoint.y, v.y);
            minPoint.z = std::min(minPoint.z, v.z);
            maxPoint.x = std::max(maxPoint.x, v.x);
            maxPoint.y = std::max(maxPoint.y, v.y);
            maxPoint.z = std::max(maxPoint.z, v.z);
        }
        localAABB = AABB(minPoint + lc, maxPoint + lc);
    }
    
    m_localVertices = vertices;
    for (auto& v : m_localVertices) {
        v = v + lc;
    }
    
    printf("[PolyhedronCollider] Created (auto-triangulated): lc=(%.2f,%.2f,%.2f), %zu vertices, %u faces\n",
           lc.x, lc.y, lc.z, vertices.size(), mesh.numFaces);
}

void PolyhedronCollider::BuildFromTriangles(const std::vector<Vec3>& vertices,
                                            const std::vector<int>& indices) {
    mesh.Clear();
    
    // 添加所有顶点
    for (const auto& v : vertices) {
        mesh.AddVert(v);
    }
    
    // 添加所有三角形面
    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        mesh.AddFace(indices[i], indices[i+1], indices[i+2]);
    }
}

void PolyhedronCollider::TriangulateConvex(const std::vector<Vec3>& vertices) {
    if (vertices.size() < 3) return;
    
    mesh.Clear();
    
    // 添加所有顶点
    for (const auto& v : vertices) {
        mesh.AddVert(v);
    }
    
    // 计算中心点
    Vec3 center(0,0,0);
    for (const auto& v : vertices) {
        center = center + v;
    }
    center = center * (1.0f / vertices.size());
    
    // 为每个面创建三角形扇
    for (size_t i = 1; i + 1 < vertices.size(); ++i) {
        mesh.AddFace(0, i, i + 1);
    }
}

Vec3 PolyhedronCollider::SupportLocal(const Vec3& direction) const {
    // 使用爬山法快速找到支撑点
    return mesh.Support(direction);
}

AABB PolyhedronCollider::GetWorldAABB() const {
    if (!body) return localAABB;
    
    if (m_useBruteForceAABB && !m_localVertices.empty()) {
        // 暴力法：遍历所有顶点（适合顶点少的网格）
        Vec3 worldMin( FLT_MAX,  FLT_MAX,  FLT_MAX);
        Vec3 worldMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);
        
        for (const auto& localVert : m_localVertices) {
            Vec3 worldVert = body->LocalToGlobal(localVert);
            worldMin.x = std::min(worldMin.x, worldVert.x);
            worldMin.y = std::min(worldMin.y, worldVert.y);
            worldMin.z = std::min(worldMin.z, worldVert.z);
            worldMax.x = std::max(worldMax.x, worldVert.x);
            worldMax.y = std::max(worldMax.y, worldVert.y);
            worldMax.z = std::max(worldMax.z, worldVert.z);
        }
        
        return AABB(worldMin, worldMax);
    } else {
        // 支撑点法：使用6个轴方向找到极值点
        const Vec3 axes[] = {
            Vec3(-1,0,0), Vec3(1,0,0),
            Vec3(0,-1,0), Vec3(0,1,0),
            Vec3(0,0,-1), Vec3(0,0,1)
        };
        
        Vec3 localAxes[6];
        for (int i = 0; i < 6; ++i) {
            localAxes[i] = body->GlobalToLocalVec(axes[i]);
        }
        
        Vec3 worldSupports[6];
        for (int i = 0; i < 6; ++i) {
            Vec3 localSupport = mesh.Support(localAxes[i]);
            worldSupports[i] = body->LocalToGlobal(localSupport);
        }
        
        // 从支撑点计算AABB
        Vec3 minPoint = worldSupports[0];
        Vec3 maxPoint = worldSupports[1];
        
        for (int i = 2; i < 6; ++i) {
            minPoint.x = std::min(minPoint.x, worldSupports[i].x);
            minPoint.y = std::min(minPoint.y, worldSupports[i].y);
            minPoint.z = std::min(minPoint.z, worldSupports[i].z);
            maxPoint.x = std::max(maxPoint.x, worldSupports[i].x);
            maxPoint.y = std::max(maxPoint.y, worldSupports[i].y);
            maxPoint.z = std::max(maxPoint.z, worldSupports[i].z);
        }
        
        // 添加小边距防止数值误差
        float margin = 0.01f;
        return AABB(minPoint - Vec3(margin, margin, margin),
                    maxPoint + Vec3(margin, margin, margin));
    }
}