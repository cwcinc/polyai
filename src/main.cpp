#include "main.h"

class Vector3 {
public:
    double x, y, z;
    
    Vector3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    
    bool equals(const Vector3& other, double epsilon = 1e-6) const {
        return std::abs(x - other.x) < epsilon && 
               std::abs(y - other.y) < epsilon && 
               std::abs(z - other.z) < epsilon;
    }
};
 
class Quaternion {
public:
    double x, y, z, w;
    
    Quaternion(double x = 0, double y = 0, double z = 0, double w = 1) : x(x), y(y), z(z), w(w) {}
    
    bool equals(const Quaternion& other, double epsilon = 1e-6) const {
        return std::abs(x - other.x) < epsilon && 
               std::abs(y - other.y) < epsilon && 
               std::abs(z - other.z) < epsilon && 
               std::abs(w - other.w) < epsilon;
    }
};

class PhysicsWorld {
private:
    std::unique_ptr<btDefaultCollisionConfiguration> collisionConfig;
    std::unique_ptr<btCollisionDispatcher> dispatcher;
    std::unique_ptr<btDbvtBroadphase> broadphase;
    std::unique_ptr<btSequentialImpulseConstraintSolver> solver;
    std::unique_ptr<btDiscreteDynamicsWorld> dynamicsWorld;
    
    struct GroundInfo {
        btRigidBody* body;
        btCollisionShape* shape;
        bool isActive;
    };
    
    std::unique_ptr<GroundInfo> ground;
    
    static const int STEPS_PER_SECOND = 1000;

public:
    PhysicsWorld() {
        collisionConfig = std::make_unique<btDefaultCollisionConfiguration>();
        dispatcher = std::make_unique<btCollisionDispatcher>(collisionConfig.get());
        broadphase = std::make_unique<btDbvtBroadphase>();
        solver = std::make_unique<btSequentialImpulseConstraintSolver>();
        dynamicsWorld = std::make_unique<btDiscreteDynamicsWorld>(
            dispatcher.get(), broadphase.get(), solver.get(), collisionConfig.get());
        
        dynamicsWorld->setGravity(btVector3(0, -9.82, 0));
    }
    
    ~PhysicsWorld() {
        dispose();
    }
    
    void dispose() {
        if (ground && ground->isActive) {
            dynamicsWorld->removeRigidBody(ground->body);
        }
        if (ground) {
            delete ground->body->getMotionState();
            delete ground->body;
            delete ground->shape;
            ground.reset();
        }
    }
    
    void createGroundPlane() {
        if (ground) {
            throw std::runtime_error("Ground is already initialized");
        }
        
        btVector3 normal(0, 1, 0);
        auto* shape = new btStaticPlaneShape(normal, 0);
        shape->setMargin(0.01);
        
        btTransform transform;
        transform.setIdentity();
        auto* motionState = new btDefaultMotionState(transform);
        
        btVector3 localInertia(0, 0, 0);
        shape->calculateLocalInertia(0, localInertia);
        
        btRigidBody::btRigidBodyConstructionInfo rbInfo(0, motionState, shape, localInertia);
        auto* body = new btRigidBody(rbInfo);
        body->setFriction(1.0);
        
        ground = std::make_unique<GroundInfo>();
        ground->body = body;
        ground->shape = shape;
        ground->isActive = false;
    }
    
    void activePhysicsAt(const Vector3& position) {
        if (ground) {
            if (position.y < 4) {
                if (!ground->isActive) {
                    dynamicsWorld->addRigidBody(ground->body);
                    ground->isActive = true;
                }
            } else if (position.y > 5 && ground->isActive) {
                dynamicsWorld->removeRigidBody(ground->body);
                ground->isActive = false;
            }
        }
    }
    
    void step() {
        dynamicsWorld->stepSimulation(1.0 / STEPS_PER_SECOND, 0, 1.0 / STEPS_PER_SECOND);
    }
    
    btDiscreteDynamicsWorld* getWorld() {
        return dynamicsWorld.get();
    }
    
    btCollisionDispatcher* getDispatcher() {
        return dispatcher.get();
    }
};

bool determinismCheck() {
    Vector3 expectedPosition(-0.6827400326728821, 0.11212741583585739, 2.6956899166107178);
    Quaternion expectedRotation(-0.615668535232544, 0.03904851898550987, 0.7859793305397034, 0.04079177975654602);

    PhysicsWorld physicsWorld;
    physicsWorld.createGroundPlane();
    physicsWorld.activePhysicsAt(Vector3(0, 0, 0));

    btTransform chassisTransform;
    chassisTransform.setIdentity();
    auto* chassisMotionState = new btDefaultMotionState(chassisTransform);
    
    btVector3 chassisInertia(0, 0, 0);
    btVector3 chassisHalfExtents(0.1, 0.1, 0.1);
    auto* chassisShape = new btBoxShape(chassisHalfExtents);
    chassisShape->calculateLocalInertia(400, chassisInertia);
    
    btRigidBody::btRigidBodyConstructionInfo chassisRbInfo(400, chassisMotionState, chassisShape, chassisInertia);
    auto* chassisBody = new btRigidBody(chassisRbInfo);
    chassisBody->setActivationState(DISABLE_DEACTIVATION);
    physicsWorld.getWorld()->addRigidBody(chassisBody);

    btRaycastVehicle::btVehicleTuning tuning;
    auto* vehicleRayCaster = new btDefaultVehicleRaycaster(physicsWorld.getWorld());
    auto* vehicle = new btRaycastVehicle(tuning, chassisBody, vehicleRayCaster);
    vehicle->setCoordinateSystem(0, 1, 2);
    physicsWorld.getWorld()->addAction(vehicle);

    btVector3 wheelDirectionCS0(0, -1, 0);
    btVector3 wheelAxleCS(-1, 0, 0);
    
    struct WheelInfo {
        const char* name;
        btVector3 connectionPoint;
        bool isFrontWheel;
    };
    
    WheelInfo wheels[] = {
        {"WheelFL", btVector3(0.627909, 0.27, 1.3478), true},
        {"WheelFR", btVector3(-0.627909, 0.27, 1.3478), true},
        {"WheelBL", btVector3(0.720832, 0.27, -1.52686), false},
        {"WheelBR", btVector3(-0.720832, 0.27, -1.52686), false}
    };
    
    for (const auto& wheel : wheels) {
        vehicle->addWheel(wheel.connectionPoint, wheelDirectionCS0, wheelAxleCS,
                         0.12, 0.331, tuning, wheel.isFrontWheel);
    }

    btTransform initialTransform;
    initialTransform.setIdentity();
    chassisBody->setWorldTransform(initialTransform);
    chassisBody->getMotionState()->setWorldTransform(initialTransform);
    
    vehicle->resetSuspension();
    vehicle->setSteeringValue(0, 0);
    vehicle->setSteeringValue(0, 1);

    btTransform secondBodyTransform;
    secondBodyTransform.setIdentity();
    auto* secondBodyMotionState = new btDefaultMotionState(secondBodyTransform);
    
    btVector3 secondBodyInertia(0, 0, 0);
    btVector3 secondBodyHalfExtents(0.1, 0.1, 0.1);
    auto* secondBodyShape = new btBoxShape(secondBodyHalfExtents);
    secondBodyShape->calculateLocalInertia(100, secondBodyInertia);
    
    btRigidBody::btRigidBodyConstructionInfo secondBodyRbInfo(100, secondBodyMotionState, secondBodyShape, secondBodyInertia);
    auto* secondBody = new btRigidBody(secondBodyRbInfo);
    secondBody->setActivationState(DISABLE_DEACTIVATION);
    physicsWorld.getWorld()->addRigidBody(secondBody);

    const float engineForce = 100000.0;
    vehicle->applyEngineForce(engineForce, 2);
    vehicle->applyEngineForce(engineForce, 3);

    for (int i = 0; i < 999; i++) {
        physicsWorld.step();
    }

    btTransform finalTransform;
    chassisBody->getMotionState()->getWorldTransform(finalTransform);
    
    btVector3 finalOrigin = finalTransform.getOrigin();
    btQuaternion finalRotation = finalTransform.getRotation();

    Vector3 actualPosition(finalOrigin.x(), finalOrigin.y(), finalOrigin.z());
    Quaternion actualRotation(finalRotation.x(), finalRotation.y(), finalRotation.z(), finalRotation.w());

    bool positionMatch = expectedPosition.equals(actualPosition);
    bool rotationMatch = expectedRotation.equals(actualRotation);

    delete vehicle;
    delete vehicleRayCaster;
    delete chassisShape;
    delete chassisBody;
    delete secondBodyShape;
    delete secondBody;

    bool result = positionMatch && rotationMatch;
    if (!result) {
        printf("Expected Position: (%f, %f, %f)\n", expectedPosition.x, expectedPosition.y, expectedPosition.z);
        printf("Actual Position:   (%f, %f, %f)\n", actualPosition.x, actualPosition.y, actualPosition.z);
        printf("Expected Rotation: (%f, %f, %f, %f)\n", expectedRotation.x, expectedRotation.y, expectedRotation.z, expectedRotation.w);
        printf("Actual Rotation:   (%f, %f, %f, %f)\n", actualRotation.x, actualRotation.y, actualRotation.z, actualRotation.w);
        std::cerr << "Determinism check failed: Simulation" << std::endl;
    }

    return result;
}

int main() {
    if (determinismCheck()) {
        std::cout << "Determinism check passed!" << std::endl;
        return 0;
    } else {
        std::cout << "Determinism check failed!" << std::endl;
        return 1;
    }
}
