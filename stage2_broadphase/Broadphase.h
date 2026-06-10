#ifndef BROADPHASE_H
#define BROADPHASE_H

#include "Collider.h"
#include <vector>
#include <utility>
#include <algorithm>

// 可能碰撞的碰撞体对
using ColliderPair = std::pair<Collider*, Collider*>;
using ColliderPairList = std::vector<ColliderPair>;

// 宽阶段基类（接口）
class Broadphase {
public:
    virtual ~Broadphase() {}
    
    virtual void AddCollider(Collider* collider) = 0;
    virtual void RemoveCollider(Collider* collider) = 0;
    virtual void Update() = 0;
    virtual const ColliderPairList& GetPotentialCollisions() = 0;
    virtual Collider* Pick(const Vec3& point) const = 0;
    virtual std::vector<Collider*> Query(const AABB& region) const = 0;
};

// N² 宽阶段（暴力双重循环）
class NSquaredBroadphase : public Broadphase {
private:
    std::vector<Collider*> m_colliders;
    ColliderPairList m_pairs;
    
public:
    virtual void AddCollider(Collider* collider) override;
    virtual void RemoveCollider(Collider* collider) override;
    virtual void Update() override;
    virtual const ColliderPairList& GetPotentialCollisions() override;
    virtual Collider* Pick(const Vec3& point) const override;
    virtual std::vector<Collider*> Query(const AABB& region) const override;
    
    size_t GetColliderCount() const { return m_colliders.size(); }
};

#endif