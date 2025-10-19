#pragma once
#include <iostream>

#include "btBulletDynamicsCommon.h"

namespace DeterminismCheck {
class Vector3 {
  public:
    double x, y, z;

    Vector3(double x = 0, double y = 0, double z = 0);
    double lengthSq() const;
    bool equals(const Vector3 &other) const;
    bool operator==(const Vector3 &other) const;
    friend std::ostream &operator<<(std::ostream &stream, const Vector3 &self);
};

class Quaternion {
  public:
    double x, y, z, w;

    Quaternion(double x = 0, double y = 0, double z = 0, double w = 1);
    bool equals(const Quaternion &other) const;
    bool operator==(const Quaternion &other) const;
    friend std::ostream &operator<<(std::ostream &stream,
                                    const Quaternion &self);
};

class PhysicsWorld {
  private:
    static constexpr int STEPS_PER_SECOND = 1000;
    static constexpr double TIME_STEP = 1.0 / STEPS_PER_SECOND;

    btDefaultCollisionConfiguration m_collisionConfiguration;
    btCollisionDispatcher m_dispatcher;
    btDbvtBroadphase m_broadphase;
    btSequentialImpulseConstraintSolver m_solver;
    btDiscreteDynamicsWorld m_dynamicsWorld;

    struct GroundInfo {
        btRigidBody *body;
        btCollisionShape *shape;
        bool isActive;
    };
    std::unique_ptr<GroundInfo> m_ground;

    struct MountainInfo {
        btRigidBody *body;
        btCollisionShape *shape;
        btTriangleMesh *triangleMesh;
        btVector3 *offset;
        double minimumRadius;
        bool isActive;
    };
    std::unique_ptr<MountainInfo> m_mountains;

  public:
    PhysicsWorld();
    ~PhysicsWorld();
    void dispose();
    void createGroundPlane();
    void createMountains(const double vertices[], const size_t verticeCount, const Vector3 &offset);
    void activePhysicsAt(const Vector3 &position);
    void step();
    btDiscreteDynamicsWorld &getWorld();
    btCollisionDispatcher &getDispatcher();
};

bool determinismCheck();
} // namespace DeterminismCheck