#ifndef POLYHEDRON_COLLIDER_H
#define POLYHEDRON_COLLIDER_H

#include "Collider.h"
#include "HalfEdge.h"
#include <vector>

// 多面体碰撞体
struct PolyhedronCollider : public Collider {
    HalfEdgeMesh mesh;  // 半边数据结构
    
    // 从顶点和索引构建多面体（三角形面）
    PolyhedronCollider(RigidBody* b, const Vec3& lc,
                       const std::vector<Vec3>& vertices,
                       const std::vector<int>& indices);
    
    // 从顶点数组构建多面体（自动三角剖分，仅限凸多面体）
    PolyhedronCollider(RigidBody* b, const Vec3& lc,
                       const std::vector<Vec3>& vertices);
    
    virtual ~PolyhedronCollider() {}
    
    virtual Vec3 SupportLocal(const Vec3& direction) const override;
    
    // 重写AABB更新以获得更好的性能
    virtual AABB GetWorldAABB() const override;
    
    // 获取信息
    unsigned int GetNumVertices() const { return mesh.GetNumVerts(); }
    unsigned int GetNumFaces() const { return mesh.GetNumFaces(); }
    
private:
    void BuildFromTriangles(const std::vector<Vec3>& vertices,
                           const std::vector<int>& indices);
    
    // 简单凸多面体的三角剖分（风扇法）
    void TriangulateConvex(const std::vector<Vec3>& vertices);
    
    // 缓存本地顶点用于暴力AABB更新
    std::vector<Vec3> m_localVertices;
    bool m_useBruteForceAABB;  // 小多面体使用暴力法更快
};

#endif