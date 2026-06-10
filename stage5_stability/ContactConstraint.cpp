#include "ContactConstraint.h"
#include <cstdio>
#include <algorithm>
#include <cmath>

ContactConstraint::ContactConstraint()
    : m_bodyA(nullptr)
    , m_bodyB(nullptr)
    , m_penetrationDepth(0.0f)
    , m_restitution(0.3f)
    , m_accumulatedNormalImpulse(0.0f)
    , m_frictionCoefficient(0.5f)
    , m_accumulatedTangentImpulse1(0.0f)
    , m_accumulatedTangentImpulse2(0.0f)
    , m_persistent(false) {
}

ContactConstraint::ContactConstraint(RigidBody* bodyA, RigidBody* bodyB,
                                   const Vec3& contactPointA, const Vec3& contactPointB,
                                   const Vec3& normal, float penetrationDepth)
    : m_bodyA(bodyA)
    , m_bodyB(bodyB)
    , m_contactPointA(contactPointA)
    , m_contactPointB(contactPointB)
    , m_normal(normal)
    , m_penetrationDepth(penetrationDepth)
    , m_restitution(0.3f)
    , m_accumulatedNormalImpulse(0.0f)
    , m_frictionCoefficient(0.5f)
    , m_accumulatedTangentImpulse1(0.0f)
    , m_accumulatedTangentImpulse2(0.0f)
    , m_persistent(false) {
    
    if (m_bodyA) {
        m_localContactA = m_bodyA->GlobalToLocal(m_contactPointA);
    } else {
        m_localContactA = m_contactPointA;
    }
    
    if (m_bodyB) {
        m_localContactB = m_bodyB->GlobalToLocal(m_contactPointB);
    } else {
        m_localContactB = m_contactPointB;
    }
    
    // 计算切向向量
    Vec3 up(0, 1, 0);
    if (fabs(m_normal.Dot(up)) > 0.99f) {
        up = Vec3(1, 0, 0);
    }
    
    m_tangent1 = m_normal.Cross(up);
    float len1 = m_tangent1.Length();
    if (len1 > 1e-6f) {
        m_tangent1 = m_tangent1 * (1.0f / len1);
    }
    
    m_tangent2 = m_normal.Cross(m_tangent1);
    float len2 = m_tangent2.Length();
    if (len2 > 1e-6f) {
        m_tangent2 = m_tangent2 * (1.0f / len2);
    }
}

void ContactConstraint::ResetAccumulatedImpulse() {
    m_accumulatedNormalImpulse = 0.0f;
    m_accumulatedTangentImpulse1 = 0.0f;
    m_accumulatedTangentImpulse2 = 0.0f;
}

void ContactConstraint::ApplyWarmStartImpulses(float factor) {
    if (!m_bodyA || !m_bodyB) return;
    if (m_bodyA->m_isStatic && m_bodyB->m_isStatic) return;
    
    if (m_accumulatedNormalImpulse == 0.0f && 
        m_accumulatedTangentImpulse1 == 0.0f && 
        m_accumulatedTangentImpulse2 == 0.0f) {
        return;
    }
    
    Vec3 rA = m_contactPointA - m_bodyA->m_globalCentroid;
    Vec3 rB = m_contactPointB - m_bodyB->m_globalCentroid;
    
    float scaledNormalImpulse = m_accumulatedNormalImpulse * factor;
    if (fabs(scaledNormalImpulse) > 1e-8f) {
        Vec3 impulse_normal = m_normal * scaledNormalImpulse;
        
        if (!m_bodyA->m_isStatic) {
            m_bodyA->m_linearVelocity -= impulse_normal * m_bodyA->m_inverseMass;
            m_bodyA->m_angularVelocity -= m_bodyA->m_globalInverseInertiaTensor * rA.Cross(impulse_normal);
        }
        
        if (!m_bodyB->m_isStatic) {
            m_bodyB->m_linearVelocity += impulse_normal * m_bodyB->m_inverseMass;
            m_bodyB->m_angularVelocity += m_bodyB->m_globalInverseInertiaTensor * rB.Cross(impulse_normal);
        }
    }
    
    float scaledTangentImpulse1 = m_accumulatedTangentImpulse1 * factor;
    if (fabs(scaledTangentImpulse1) > 1e-8f) {
        Vec3 impulse_tangent1 = m_tangent1 * scaledTangentImpulse1;
        
        if (!m_bodyA->m_isStatic) {
            m_bodyA->m_linearVelocity -= impulse_tangent1 * m_bodyA->m_inverseMass;
            m_bodyA->m_angularVelocity -= m_bodyA->m_globalInverseInertiaTensor * rA.Cross(impulse_tangent1);
        }
        
        if (!m_bodyB->m_isStatic) {
            m_bodyB->m_linearVelocity += impulse_tangent1 * m_bodyB->m_inverseMass;
            m_bodyB->m_angularVelocity += m_bodyB->m_globalInverseInertiaTensor * rB.Cross(impulse_tangent1);
        }
    }
    
    float scaledTangentImpulse2 = m_accumulatedTangentImpulse2 * factor;
    if (fabs(scaledTangentImpulse2) > 1e-8f) {
        Vec3 impulse_tangent2 = m_tangent2 * scaledTangentImpulse2;
        
        if (!m_bodyA->m_isStatic) {
            m_bodyA->m_linearVelocity -= impulse_tangent2 * m_bodyA->m_inverseMass;
            m_bodyA->m_angularVelocity -= m_bodyA->m_globalInverseInertiaTensor * rA.Cross(impulse_tangent2);
        }
        
        if (!m_bodyB->m_isStatic) {
            m_bodyB->m_linearVelocity += impulse_tangent2 * m_bodyB->m_inverseMass;
            m_bodyB->m_angularVelocity += m_bodyB->m_globalInverseInertiaTensor * rB.Cross(impulse_tangent2);
        }
    }
}

void ContactConstraint::Solve(float dt, float restitutionSlop) {
    if (!m_bodyA || !m_bodyB) return;
    if (m_bodyA->m_isStatic && m_bodyB->m_isStatic) return;
    
    // 如果穿透深度很小，跳过速度修正
    if (m_penetrationDepth < 0.001f) return;
    
    Vec3 rA = m_contactPointA - m_bodyA->m_globalCentroid;
    Vec3 rB = m_contactPointB - m_bodyB->m_globalCentroid;
    
    // ============================================
    // 1. 法向约束求解
    // ============================================
    
    Vec3 velA = m_bodyA->m_linearVelocity + m_bodyA->m_angularVelocity.Cross(rA);
    Vec3 velB = m_bodyB->m_linearVelocity + m_bodyB->m_angularVelocity.Cross(rB);
    Vec3 relVel = velB - velA;
    float normalRelVel = relVel.Dot(m_normal);
    
    Vec3 rA_cross_n = rA.Cross(m_normal);
    Vec3 rB_cross_n = rB.Cross(m_normal);
    
    float invMassSum = m_bodyA->m_inverseMass + m_bodyB->m_inverseMass;
    float invInertiaA = rA_cross_n.Dot(m_bodyA->m_globalInverseInertiaTensor * rA_cross_n);
    float invInertiaB = rB_cross_n.Dot(m_bodyB->m_globalInverseInertiaTensor * rB_cross_n);
    float K_normal = invMassSum + invInertiaA + invInertiaB;
    
    if (K_normal < 1e-8f) return;
    
    // 速度容差：低于此值认为物体已静止，不应用弹性
    float effectiveRestitution = m_restitution;
    if (normalRelVel > -restitutionSlop) {
        effectiveRestitution = 0.0f;
    }
    
    float lambda_normal = -(1.0f + effectiveRestitution) * normalRelVel / K_normal;
    
    float oldNormalImpulse = m_accumulatedNormalImpulse;
    m_accumulatedNormalImpulse = std::max(oldNormalImpulse + lambda_normal, 0.0f);
    lambda_normal = m_accumulatedNormalImpulse - oldNormalImpulse;
    
    if (fabs(lambda_normal) > 1e-8f) {
        Vec3 impulse_normal = m_normal * lambda_normal;
        
        if (!m_bodyA->m_isStatic) {
            m_bodyA->m_linearVelocity -= impulse_normal * m_bodyA->m_inverseMass;
            m_bodyA->m_angularVelocity -= m_bodyA->m_globalInverseInertiaTensor * rA.Cross(impulse_normal);
        }
        
        if (!m_bodyB->m_isStatic) {
            m_bodyB->m_linearVelocity += impulse_normal * m_bodyB->m_inverseMass;
            m_bodyB->m_angularVelocity += m_bodyB->m_globalInverseInertiaTensor * rB.Cross(impulse_normal);
        }
    }
    
    // ============================================
    // 2. 切向约束求解（摩擦力）
    // ============================================
    
    velA = m_bodyA->m_linearVelocity + m_bodyA->m_angularVelocity.Cross(rA);
    velB = m_bodyB->m_linearVelocity + m_bodyB->m_angularVelocity.Cross(rB);
    relVel = velB - velA;
    
    float tangentRelVel1 = relVel.Dot(m_tangent1);
    float tangentRelVel2 = relVel.Dot(m_tangent2);
    
    Vec3 rA_cross_t1 = rA.Cross(m_tangent1);
    Vec3 rB_cross_t1 = rB.Cross(m_tangent1);
    Vec3 rA_cross_t2 = rA.Cross(m_tangent2);
    Vec3 rB_cross_t2 = rB.Cross(m_tangent2);
    
    float invInertiaA_t1 = rA_cross_t1.Dot(m_bodyA->m_globalInverseInertiaTensor * rA_cross_t1);
    float invInertiaB_t1 = rB_cross_t1.Dot(m_bodyB->m_globalInverseInertiaTensor * rB_cross_t1);
    float invInertiaA_t2 = rA_cross_t2.Dot(m_bodyA->m_globalInverseInertiaTensor * rA_cross_t2);
    float invInertiaB_t2 = rB_cross_t2.Dot(m_bodyB->m_globalInverseInertiaTensor * rB_cross_t2);
    
    float K_tangent1 = invMassSum + invInertiaA_t1 + invInertiaB_t1;
    float K_tangent2 = invMassSum + invInertiaA_t2 + invInertiaB_t2;
    
    if (K_tangent1 < 1e-8f || K_tangent2 < 1e-8f) return;
    
    float lambda_tangent1 = -tangentRelVel1 / K_tangent1;
    float lambda_tangent2 = -tangentRelVel2 / K_tangent2;
    
    float maxFriction = m_frictionCoefficient * m_accumulatedNormalImpulse;
    
    float oldTangentImpulse1 = m_accumulatedTangentImpulse1;
    float oldTangentImpulse2 = m_accumulatedTangentImpulse2;
    
    m_accumulatedTangentImpulse1 += lambda_tangent1;
    m_accumulatedTangentImpulse2 += lambda_tangent2;
    
    float frictionImpulseMag = sqrtf(m_accumulatedTangentImpulse1 * m_accumulatedTangentImpulse1 + 
                                      m_accumulatedTangentImpulse2 * m_accumulatedTangentImpulse2);
    if (frictionImpulseMag > maxFriction && maxFriction > 0) {
        float scale = maxFriction / frictionImpulseMag;
        m_accumulatedTangentImpulse1 *= scale;
        m_accumulatedTangentImpulse2 *= scale;
    }
    
    lambda_tangent1 = m_accumulatedTangentImpulse1 - oldTangentImpulse1;
    lambda_tangent2 = m_accumulatedTangentImpulse2 - oldTangentImpulse2;
    
    if (fabs(lambda_tangent1) > 1e-8f) {
        Vec3 impulse_tangent1 = m_tangent1 * lambda_tangent1;
        
        if (!m_bodyA->m_isStatic) {
            m_bodyA->m_linearVelocity -= impulse_tangent1 * m_bodyA->m_inverseMass;
            m_bodyA->m_angularVelocity -= m_bodyA->m_globalInverseInertiaTensor * rA.Cross(impulse_tangent1);
        }
        
        if (!m_bodyB->m_isStatic) {
            m_bodyB->m_linearVelocity += impulse_tangent1 * m_bodyB->m_inverseMass;
            m_bodyB->m_angularVelocity += m_bodyB->m_globalInverseInertiaTensor * rB.Cross(impulse_tangent1);
        }
    }
    
    if (fabs(lambda_tangent2) > 1e-8f) {
        Vec3 impulse_tangent2 = m_tangent2 * lambda_tangent2;
        
        if (!m_bodyA->m_isStatic) {
            m_bodyA->m_linearVelocity -= impulse_tangent2 * m_bodyA->m_inverseMass;
            m_bodyA->m_angularVelocity -= m_bodyA->m_globalInverseInertiaTensor * rA.Cross(impulse_tangent2);
        }
        
        if (!m_bodyB->m_isStatic) {
            m_bodyB->m_linearVelocity += impulse_tangent2 * m_bodyB->m_inverseMass;
            m_bodyB->m_angularVelocity += m_bodyB->m_globalInverseInertiaTensor * rB.Cross(impulse_tangent2);
        }
    }
}

void ContactConstraint::PositionalCorrection(float penetrationSlop) {
    if (!m_bodyA || !m_bodyB) return;
    
    const float percent = 0.9f;      // 90% 修正
    const float slop = penetrationSlop;
    
    float correctionDepth = m_penetrationDepth - slop;
    if (correctionDepth <= 0.0f) return;
    
    // 限制最大修正量，防止过度修正
    float maxCorrection = 0.1f;
    if (correctionDepth > maxCorrection) {
        correctionDepth = maxCorrection;
    }
    
    float correction = correctionDepth * percent;
    Vec3 correctionVec = m_normal * correction;
    
    float invMassSum = m_bodyA->m_inverseMass + m_bodyB->m_inverseMass;
    if (invMassSum < 1e-8f) return;
    
    if (!m_bodyA->m_isStatic) {
        float weightA = m_bodyA->m_inverseMass / invMassSum;
        m_bodyA->m_position -= correctionVec * weightA;
        m_bodyA->UpdateGlobalCentroidFromPosition();
    }
    
    if (!m_bodyB->m_isStatic) {
        float weightB = m_bodyB->m_inverseMass / invMassSum;
        m_bodyB->m_position += correctionVec * weightB;
        m_bodyB->UpdateGlobalCentroidFromPosition();
    }
}

void ContactConstraint::SetFrictionCoefficient(float friction) {
    m_frictionCoefficient = friction;
}