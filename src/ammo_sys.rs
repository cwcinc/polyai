#![allow(unsafe_op_in_unsafe_fn)]
#![allow(improper_ctypes)]
use autocxx::prelude::*;

include_cpp! {
    #include "ammo.h"
    safety!(unsafe)

    block!("btCollisionObjectWrapper")

    generate!("btBvhTriangleMeshShape")
    generate!("btCollisionDispatcher")
    generate!("btConvexHullShape")
    generate!("btDbvtBroadphase")
    generate!("btDefaultCollisionConfiguration")
    generate!("btDefaultMotionState")
    generate!("btDefaultVehicleRaycaster")
    generate!("btDiscreteDynamicsWorld")
    generate!("btQuaternion")
    generate!("btRaycastVehicle")
    generate!("btRigidBody")
    generate!("btSequentialImpulseConstraintSolver")
    generate!("btStaticPlaneShape")
    generate!("btTransform")
    generate!("btTriangleMesh")
    generate!("btVector3")
    generate!("btVehicleTuning")
}

pub use self::ffi::*;
