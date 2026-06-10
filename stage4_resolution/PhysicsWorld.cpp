#include "PhysicsWorld.h"
#include "GJK.h"
#include "EPA.h"
#include <cstdio>
#include <algorithm>
#include <cfloat>

static bool BoxBoxCollision(BoxCollider* boxA, BoxCollider* boxB, ContactManifold* manifold);

PhysicsWorld::PhysicsWorld(const Vec3& gravity)
    : m_gravity(gravity)
    , m_numIterations(10)
    , m_enableFriction(true)
    , m_defaultRestitution(0.5f)
    , m_defaultFriction(0.5f) {
    
    m_broadphase = new NSquaredBroadphase();
}

PhysicsWorld::~PhysicsWorld() {
    delete m_broadphase;
    for (RigidBody* body : m_bodies) {
        delete body;
    }
}

void PhysicsWorld::AddRigidBody(RigidBody* body) {
    if (body) {
        m_bodies.push_back(body);
        for (Collider* collider : body->m_colliders) {
            m_broadphase->AddCollider(collider);
        }
    }
}

void PhysicsWorld::RemoveRigidBody(RigidBody* body) {
    auto it = std::find(m_bodies.begin(), m_bodies.end(), body);
    if (it != m_bodies.end()) {
        for (Collider* collider : body->m_colliders) {
            m_broadphase->RemoveCollider(collider);
        }
        m_bodies.erase(it);
    }
}

static bool BoxBoxCollision(BoxCollider* boxA, BoxCollider* boxB, ContactManifold* manifold) {
    if (!boxA || !boxB) return false;
    
    RigidBody* bodyA = boxA->body;
    RigidBody* bodyB = boxB->body;
    
    Vec3 centerA = bodyA->LocalToGlobal(boxA->localCentroid);
    Vec3 centerB = bodyB->LocalToGlobal(boxB->localCentroid);
    
    Mat3 rotA = bodyA->m_orientation;
    Mat3 rotB = bodyB->m_orientation;
    
    Vec3 axesA[3] = {
        rotA * Vec3(1,0,0),
        rotA * Vec3(0,1,0),
        rotA * Vec3(0,0,1)
    };
    
    Vec3 axesB[3] = {
        rotB * Vec3(1,0,0),
        rotB * Vec3(0,1,0),
        rotB * Vec3(0,0,1)
    };
    
    Vec3 halfA = boxA->halfExtents;
    Vec3 halfB = boxB->halfExtents;
    
    float minOverlap = FLT_MAX;
    Vec3 bestAxis(0,1,0);
    
    Vec3 testAxes[15];
    int axisCount = 0;
    
    for (int i = 0; i < 3; i++) testAxes[axisCount++] = axesA[i];
    for (int i = 0; i < 3; i++) testAxes[axisCount++] = axesB[i];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Vec3 cross = axesA[i].Cross(axesB[j]);
            if (cross.LengthSq() > 0.01f) {
                testAxes[axisCount++] = cross.Normalized();
            }
        }
    }
    
    for (int i = 0; i < axisCount; i++) {
        Vec3 axis = testAxes[i];
        float len = axis.Length();
        if (len < 1e-6f) continue;
        axis = axis * (1.0f / len);
        
        float projA = fabs(axesA[0].Dot(axis)) * halfA.x +
                      fabs(axesA[1].Dot(axis)) * halfA.y +
                      fabs(axesA[2].Dot(axis)) * halfA.z;
        float projB = fabs(axesB[0].Dot(axis)) * halfB.x +
                      fabs(axesB[1].Dot(axis)) * halfB.y +
                      fabs(axesB[2].Dot(axis)) * halfB.z;
        
        float centerDist = fabs((centerB - centerA).Dot(axis));
        float overlap = projA + projB - centerDist;
        
        if (overlap < 0) return false;
        
        if (overlap < minOverlap) {
            minOverlap = overlap;
            bestAxis = axis;
        }
    }
    
    if ((centerB - centerA).Dot(bestAxis) < 0) {
        bestAxis = -bestAxis;
    }
    
    manifold->penetrationDepth = minOverlap;
    manifold->normal = bestAxis;
    manifold->localContactA = boxA->localCentroid;
    manifold->localContactB = boxB->localCentroid;
    manifold->contactPointA = centerA;
    manifold->contactPointB = centerB;
    
    return true;
}

void PhysicsWorld::GenerateConstraints() {
    m_constraints.clear();
    
    const auto& pairs = m_broadphase->GetPotentialCollisions();
    
    for (const auto& pair : pairs) {
        Collider* colliderA = pair.first;
        Collider* colliderB = pair.second;
        
        if (colliderA->body->m_isStatic && colliderB->body->m_isStatic) continue;
        
        GJKResult gjkResult;
        if (GJK(colliderA, colliderB, &gjkResult) && gjkResult.isColliding) {
            ContactManifold manifold;
            bool success = false;
            
            if (EPA(colliderA, colliderB, gjkResult.simplex, &manifold)) {
                success = true;
            } 
            else if (colliderA->type == COLLIDER_BOX && colliderB->type == COLLIDER_BOX) {
                success = BoxBoxCollision(
                    static_cast<BoxCollider*>(colliderA),
                    static_cast<BoxCollider*>(colliderB),
                    &manifold
                );
            }
            
            if (success) {
                ContactConstraint constraint(
                    colliderA->body, colliderB->body,
                    manifold.contactPointA, manifold.contactPointB,
                    manifold.normal, manifold.penetrationDepth
                );
                constraint.SetRestitution(m_defaultRestitution);
                constraint.SetFrictionCoefficient(m_defaultFriction);
                m_constraints.push_back(constraint);
            }
        }
    }
}

void PhysicsWorld::SolveConstraints(float dt) {
    if (m_constraints.empty()) return;
    
    for (auto& constraint : m_constraints) {
        constraint.ResetAccumulatedImpulse();
    }
    
    for (int iteration = 0; iteration < m_numIterations; ++iteration) {
        for (auto& constraint : m_constraints) {
            constraint.Solve(dt);
        }
    }
}

void PhysicsWorld::ApplyPositionalCorrection() {
    for (auto& constraint : m_constraints) {
        constraint.PositionalCorrection();
    }
}

void PhysicsWorld::Step(float dt) {
    // 1. 应用重力
    for (RigidBody* body : m_bodies) {
        if (!body->m_isStatic) {
            Vec3 gravityForce = m_gravity * body->m_mass;
            body->ApplyForce(gravityForce, body->m_globalCentroid);
        }
    }
    
    // 2. 积分
    for (RigidBody* body : m_bodies) {
        body->Integrate(dt);
    }
    
    // 3. 多次碰撞解析迭代
    const int collisionIterations = 8;
    
    for (int iter = 0; iter < collisionIterations; ++iter) {
        for (RigidBody* body : m_bodies) {
            for (Collider* collider : body->m_colliders) {
                collider->GetWorldAABB();
            }
        }
        m_broadphase->Update();
        
        GenerateConstraints();
        
        if (!m_constraints.empty()) {
            SolveConstraints(dt);
            
            for (int posIter = 0; posIter < 10; ++posIter) {
                ApplyPositionalCorrection();
                for (RigidBody* body : m_bodies) {
                    body->UpdateGlobalCentroidFromPosition();
                }
            }
        }
        
        m_constraints.clear();
    }
    
    // 4. 最终地面修正
    for (RigidBody* body : m_bodies) {
        if (!body->m_isStatic) {
            if (body->m_position.y < -4.5f) {
                body->m_position.y = -4.5f;
                body->UpdateGlobalCentroidFromPosition();
                if (body->m_linearVelocity.y < 0) {
                    body->m_linearVelocity.y = 0;
                }
            }
        }
        body->UpdateOrientationAndInertia();
    }
}