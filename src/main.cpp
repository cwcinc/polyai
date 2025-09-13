#include "main.h"

#include <cstdint>
#include <ostream>

class Vector3 {
  public:
    double x, y, z;

    Vector3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    bool equals(const Vector3 &other, double epsilon) const {
        return std::abs(x - other.x) < epsilon &&
               std::abs(y - other.y) < epsilon &&
               std::abs(z - other.z) < epsilon;
    }

    bool operator==(const Vector3 &other) const {
        return this->equals(other, 1e-6);
    }

    friend std::ostream &operator<<(std::ostream &stream, const Vector3 &self) {
        return stream << "(" << self.x << ", " << self.y << ", " << self.z
                      << ")";
    }
};

class Quaternion {
  public:
    double x, y, z, w;

    Quaternion(double x = 0, double y = 0, double z = 0, double w = 1)
        : x(x), y(y), z(z), w(w) {}

    bool equals(const Quaternion &other, double epsilon) const {
        return std::abs(x - other.x) < epsilon &&
               std::abs(y - other.y) < epsilon &&
               std::abs(z - other.z) < epsilon &&
               std::abs(w - other.w) < epsilon;
    }

    bool operator==(const Quaternion &other) const {
        return this->equals(other, 1e-6);
    }

    friend std::ostream &operator<<(std::ostream &stream,
                                    const Quaternion &self) {
        return stream << self.w << " + " << self.x << "i + " << self.y << "j + "
                      << self.z << "k";
    }
};

class PhysicsWorld {
  private:
    btDefaultCollisionConfiguration m_collisionConfig;
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

    static const uint32_t STEPS_PER_SECOND = 1000;

  public:
    PhysicsWorld()
        : m_collisionConfig(), m_dispatcher(&this->m_collisionConfig),
          m_broadphase(), m_solver(),
          m_dynamicsWorld(&this->m_dispatcher, &this->m_broadphase,
                          &this->m_solver, &this->m_collisionConfig) {
        // this->collisionConfig = btDefaultCollisionConfiguration();
        // this->dispatcher = btCollisionDispatcher(&this->collisionConfig);
        // this->broadphase = btDbvtBroadphase();
        // this->solver = btSequentialImpulseConstraintSolver();
        // this->dynamicsWorld =
        //     btDiscreteDynamicsWorld(&this->dispatcher, &this->broadphase,
        //                             &this->solver, &this->collisionConfig);

        this->m_dynamicsWorld.setGravity(btVector3(0, -9.82, 0));
    }

    ~PhysicsWorld() { dispose(); }

    void dispose() {
        if (this->m_ground && this->m_ground->isActive) {
            this->m_dynamicsWorld.removeRigidBody(m_ground->body);
        }
        if (this->m_ground) {
            delete this->m_ground->body->getMotionState();
            delete this->m_ground->body;
            delete this->m_ground->shape;
            this->m_ground.reset();
        }
    }

    void createGroundPlane() {
        if (m_ground) {
            throw std::runtime_error("Ground is already initialized");
        }

        btVector3 normal(0, 1, 0);
        auto shape = new btStaticPlaneShape(normal, 0);
        shape->setMargin(0.01);

        btTransform transform;
        transform.setIdentity();
        auto motionState = new btDefaultMotionState(transform);

        btVector3 localInertia(0, 0, 0);
        shape->calculateLocalInertia(0, localInertia);

        btRigidBody::btRigidBodyConstructionInfo rbInfo(0, motionState, shape,
                                                        localInertia);
        auto body = new btRigidBody(rbInfo);
        body->setFriction(1.0);

        this->m_ground = std::make_unique<GroundInfo>();
        this->m_ground->body = body;
        this->m_ground->shape = shape;
        this->m_ground->isActive = false;
    }

    void activePhysicsAt(const Vector3 &position) {
        if (this->m_ground) {
            if (position.y < 4) {
                if (!this->m_ground->isActive) {
                    this->m_dynamicsWorld.addRigidBody(m_ground->body);
                    this->m_ground->isActive = true;
                }
            } else if (position.y > 5 && this->m_ground->isActive) {
                this->m_dynamicsWorld.removeRigidBody(m_ground->body);
                this->m_ground->isActive = false;
            }
        }
    }

    void step() {
        this->m_dynamicsWorld.stepSimulation(1.0 / STEPS_PER_SECOND, 0,
                                             1.0 / STEPS_PER_SECOND);
    }

    btDiscreteDynamicsWorld &getWorld() { return this->m_dynamicsWorld; }
    btCollisionDispatcher &getDispatcher() { return this->m_dispatcher; }
};

bool determinismCheck() {
    Vector3 expectedPosition(-0.6827400326728821, 0.11212741583585739,
                             2.6956899166107178);
    Quaternion expectedRotation(-0.615668535232544, 0.03904851898550987,
                                0.7859793305397034, 0.04079177975654602);

    PhysicsWorld physicsWorld;
    physicsWorld.createGroundPlane();
    physicsWorld.activePhysicsAt(Vector3(0, 0, 0));

    btTransform chassisTransform;
    chassisTransform.setIdentity();
    auto chassisMotionState = btDefaultMotionState(chassisTransform);

    btVector3 chassisInertia(0, 0, 0);
    btVector3 chassisHalfExtents(0.1, 0.1, 0.1);
    auto chassisShape = btBoxShape(chassisHalfExtents);
    chassisShape.calculateLocalInertia(400, chassisInertia);

    btRigidBody::btRigidBodyConstructionInfo chassisRbInfo(
        400, &chassisMotionState, &chassisShape, chassisInertia);
    auto chassisBody = btRigidBody(chassisRbInfo);
    chassisBody.setActivationState(DISABLE_DEACTIVATION);
    physicsWorld.getWorld().addRigidBody(&chassisBody);

    btRaycastVehicle::btVehicleTuning tuning;
    auto vehicleRayCaster = btDefaultVehicleRaycaster(&physicsWorld.getWorld());
    auto vehicle = btRaycastVehicle(tuning, &chassisBody, &vehicleRayCaster);
    vehicle.setCoordinateSystem(0, 1, 2);
    physicsWorld.getWorld().addAction(&vehicle);

    btVector3 wheelDirectionCS0(0, -1, 0);
    btVector3 wheelAxleCS(-1, 0, 0);

    struct WheelInfo {
        const char *name;
        btVector3 connectionPoint;
        bool isFrontWheel;
    };

    WheelInfo wheels[] = {
        {"WheelFL", btVector3(0.627909, 0.27, 1.3478), true},
        {"WheelFR", btVector3(-0.627909, 0.27, 1.3478), true},
        {"WheelBL", btVector3(0.720832, 0.27, -1.52686), false},
        {"WheelBR", btVector3(-0.720832, 0.27, -1.52686), false}};

    for (const auto &wheel : wheels) {
        vehicle.addWheel(wheel.connectionPoint, wheelDirectionCS0, wheelAxleCS,
                         0.12, 0.331, tuning, wheel.isFrontWheel);
    }

    btTransform initialTransform;
    initialTransform.setIdentity();
    chassisBody.setWorldTransform(initialTransform);
    chassisBody.getMotionState()->setWorldTransform(initialTransform);

    vehicle.resetSuspension();
    vehicle.setSteeringValue(0, 0);
    vehicle.setSteeringValue(0, 1);

    btTransform secondBodyTransform;
    secondBodyTransform.setIdentity();
    auto secondBodyMotionState = btDefaultMotionState(secondBodyTransform);

    btVector3 secondBodyInertia(0, 0, 0);
    btVector3 secondBodyHalfExtents(0.1, 0.1, 0.1);
    auto secondBodyShape = btBoxShape(secondBodyHalfExtents);
    secondBodyShape.calculateLocalInertia(100, secondBodyInertia);

    btRigidBody::btRigidBodyConstructionInfo secondBodyRbInfo(
        100, &secondBodyMotionState, &secondBodyShape, secondBodyInertia);
    auto secondBody = btRigidBody(secondBodyRbInfo);
    secondBody.setActivationState(DISABLE_DEACTIVATION);
    physicsWorld.getWorld().addRigidBody(&secondBody);

    const float engineForce = 100000.0;
    vehicle.applyEngineForce(engineForce, 2);
    vehicle.applyEngineForce(engineForce, 3);

    for (int i = 0; i < 999; i++) {
        physicsWorld.step();
        btTransform chassisWorldTransform;
        chassisBody.getMotionState()->getWorldTransform(chassisWorldTransform);
        std::cout << "Step " << i << ": Chassis at (" << std::hexfloat
                  << std::hexfloat << chassisWorldTransform.getOrigin().x() << ", "
                  << std::hexfloat << chassisWorldTransform.getOrigin().y() << ", "
                  << std::hexfloat << chassisWorldTransform.getOrigin().z() << ")\n";
    }

    btTransform finalTransform;
    chassisBody.getMotionState()->getWorldTransform(finalTransform);

    btVector3 finalOrigin = finalTransform.getOrigin();
    btQuaternion finalRotation = finalTransform.getRotation();

    Vector3 actualPosition(finalOrigin.x(), finalOrigin.y(), finalOrigin.z());
    Quaternion actualRotation(finalRotation.x(), finalRotation.y(),
                              finalRotation.z(), finalRotation.w());

    bool positionMatch = expectedPosition == actualPosition;
    bool rotationMatch = expectedRotation == actualRotation;

    bool result = positionMatch && rotationMatch;
    if (!result) {
        // printf("Expected Position: (%f, %f, %f)\n", expectedPosition.x,
        //        expectedPosition.y, expectedPosition.z);
        std::cout << "Expected Position: " << expectedPosition << '\n';
        // printf("Actual Position:   (%f, %f, %f)\n", actualPosition.x,
        //        actualPosition.y, actualPosition.z);
        std::cout << "Actual Position: " << actualPosition << '\n';
        // printf("Expected Rotation: (%f, %f, %f, %f)\n", expectedRotation.x,
        //        expectedRotation.y, expectedRotation.z, expectedRotation.w);
        std::cout << "Expected Rotation: " << expectedRotation << '\n';
        // printf("Actual Rotation:   (%f, %f, %f, %f)\n", actualRotation.x,
        //        actualRotation.y, actualRotation.z, actualRotation.w);
        std::cout << "Actual Rotation: " << actualRotation << '\n';
        std::cerr << "Determinism check failed: Simulation\n";
    }

    return result;
}

int main() {
    if (determinismCheck()) {
        std::cout << "Determinism check passed!\n";
        return 0;
    } else {
        std::cout << "Determinism check failed!\n";
        return 1;
    }
}
