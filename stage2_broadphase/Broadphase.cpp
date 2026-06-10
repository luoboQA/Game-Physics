#include "Broadphase.h"
#include <cstdio>

void NSquaredBroadphase::AddCollider(Collider* collider) {
    if (collider) {
        m_colliders.push_back(collider);
        printf("[Broadphase] Added collider, total: %zu\n", m_colliders.size());
    }
}

void NSquaredBroadphase::RemoveCollider(Collider* collider) {
    auto it = std::find(m_colliders.begin(), m_colliders.end(), collider);
    if (it != m_colliders.end()) {
        m_colliders.erase(it);
        printf("[Broadphase] Removed collider, total: %zu\n", m_colliders.size());
    }
}

void NSquaredBroadphase::Update() {
    // N² 宽阶段不需要预处理
}

const ColliderPairList& NSquaredBroadphase::GetPotentialCollisions() {
    m_pairs.clear();
    
    for (size_t i = 0; i < m_colliders.size(); ++i) {
        Collider* a = m_colliders[i];
        if (!a || !a->body) continue;
        
        AABB aabbA = a->GetWorldAABB();
        
        for (size_t j = i + 1; j < m_colliders.size(); ++j) {
            Collider* b = m_colliders[j];
            if (!b || !b->body) continue;
            if (a->body == b->body) continue;
            
            AABB aabbB = b->GetWorldAABB();
            if (aabbA.Intersects(aabbB)) {
                m_pairs.emplace_back(a, b);
            }
        }
    }
    
    return m_pairs;
}

Collider* NSquaredBroadphase::Pick(const Vec3& point) const {
    for (Collider* c : m_colliders) {
        if (c && c->GetWorldAABB().Contains(point)) {
            return c;
        }
    }
    return nullptr;
}

std::vector<Collider*> NSquaredBroadphase::Query(const AABB& region) const {
    std::vector<Collider*> result;
    for (Collider* c : m_colliders) {
        if (c && c->GetWorldAABB().Intersects(region)) {
            result.push_back(c);
        }
    }
    return result;
}