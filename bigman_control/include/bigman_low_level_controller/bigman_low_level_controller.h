#ifndef BIGMAN_LOW_LEVEL_CONTROLLER_H
#define BIGMAN_LOW_LEVEL_CONTROLLER_H

#include "ros/ros.h"
#include "ros/time.h"
#include <std_srvs/Empty.h>
#include <ecl/geometry.hpp>
#include "bigman_driver/bigman_driver.h"
#include "bigman_kinematic/bigman_kinematic.h"
#include "quadprog/quadprog.hpp"

//todo: remove dependency on bigman_driver
//todo: chenge low level controller to be independent
//todo: as much as possible
//todo: think about: torque = f(model, state) type

enum ContactState {
    noSupport, leftSupport, rightSupport, doubleSupport
};

struct HighLevelCommand {
    // CoM reference
    RigidBodyDynamics::Math::Vector3d CoMPosRef;
    RigidBodyDynamics::Math::Vector3d CoMVelRef;
    RigidBodyDynamics::Math::Vector3d CoMAccRef;

    // LFoot reference
    RigidBodyDynamics::Math::Vector3d LFootPosRef;
    RigidBodyDynamics::Math::Matrix3d LFootRotRef;
    RigidBodyDynamics::Math::Vector3d LFootVelLinearRef;
    RigidBodyDynamics::Math::Vector3d LFootVelAngularRef;
    RigidBodyDynamics::Math::Vector3d LFootAccLinearRef;
    RigidBodyDynamics::Math::Vector3d LFootAccAngularRef;

    // RFoot reference
    RigidBodyDynamics::Math::Vector3d RFootPosRef;
    RigidBodyDynamics::Math::Matrix3d RFootRotRef;
    RigidBodyDynamics::Math::Vector3d RFootVelLinearRef;
    RigidBodyDynamics::Math::Vector3d RFootVelAngularRef;
    RigidBodyDynamics::Math::Vector3d RFootAccLinearRef;
    RigidBodyDynamics::Math::Vector3d RFootAccAngularRef;

    // contact state
    unsigned int contactState;

    // constructor
    HighLevelCommand() {
        CoMPosRef = RigidBodyDynamics::Math::Vector3d::Zero();
        CoMVelRef = RigidBodyDynamics::Math::Vector3d::Zero();
        CoMAccRef = RigidBodyDynamics::Math::Vector3d::Zero();

        LFootPosRef = RigidBodyDynamics::Math::Vector3d::Zero();
        LFootRotRef = RigidBodyDynamics::Math::Matrix3d::Identity();
        LFootVelLinearRef = RigidBodyDynamics::Math::Vector3d::Zero();
        LFootVelAngularRef = RigidBodyDynamics::Math::Vector3d::Zero();
        LFootAccLinearRef = RigidBodyDynamics::Math::Vector3d::Zero();
        LFootAccAngularRef = RigidBodyDynamics::Math::Vector3d::Zero();

        RFootPosRef = RigidBodyDynamics::Math::Vector3d::Zero();
        RFootRotRef = RigidBodyDynamics::Math::Matrix3d::Identity();
        RFootVelLinearRef = RigidBodyDynamics::Math::Vector3d::Zero();
        RFootVelAngularRef = RigidBodyDynamics::Math::Vector3d::Zero();
        RFootAccLinearRef = RigidBodyDynamics::Math::Vector3d::Zero();
        RFootAccAngularRef = RigidBodyDynamics::Math::Vector3d::Zero();

        contactState = doubleSupport;
    }
};

struct Task {

    std::string taskName;
    double weight;
    RigidBodyDynamics::Math::MatrixNd A;
    RigidBodyDynamics::Math::VectorNd b;

    Task(std::string taskName, double weight, RigidBodyDynamics::Math::MatrixNd A, RigidBodyDynamics::Math::VectorNd b)
            : taskName(taskName), weight(weight), A(A), b(b) {}

    void disp() {
        std::cout << "---------------------------------------" << std::endl;
        std::cout << taskName << " with weight: " << weight << std::endl;
        std::cout << "A: " << "(" << A.rows() << "," << A.cols() << ")" << std::endl << A << std::endl;
        std::cout << "b.T: " << std::endl << b.transpose() << std::endl;
    }
};


class LowLevelController {

public:
    ros::NodeHandle nh;
    BigmanDriver driver;


    // controller states
    RigidBodyDynamics::Math::MatrixNd inertiaMatrix;
    RigidBodyDynamics::Math::VectorNd nonlinearEffects;

    RigidBodyDynamics::Math::SpatialVector world_Acc_lSole;
    RigidBodyDynamics::Math::SpatialVector world_Acc_rSole;
    RigidBodyDynamics::Math::SpatialVector world_Acc_lHand;
    RigidBodyDynamics::Math::SpatialVector world_Acc_rHand;

    RigidBodyDynamics::Math::MatrixNd world_J_lSole;
    RigidBodyDynamics::Math::MatrixNd world_J_rSole;
    RigidBodyDynamics::Math::MatrixNd world_J_lHand;
    RigidBodyDynamics::Math::MatrixNd world_J_rHand;

    RigidBodyDynamics::Math::Vector3d world_Position_lSole;
    RigidBodyDynamics::Math::Vector3d world_Position_rSole;
    RigidBodyDynamics::Math::Vector3d world_Position_lHand;
    RigidBodyDynamics::Math::Vector3d world_Position_rHand;

    RigidBodyDynamics::Math::Matrix3d world_Rot_lSole;
    RigidBodyDynamics::Math::Matrix3d world_Rot_rSole;
    RigidBodyDynamics::Math::Matrix3d world_Rot_lHand;
    RigidBodyDynamics::Math::Matrix3d world_Rot_rHand;

    RigidBodyDynamics::Math::SpatialVector world_Velocity_lSole;
    RigidBodyDynamics::Math::SpatialVector world_Velocity_rSole;
    RigidBodyDynamics::Math::SpatialVector world_Velocity_lHand;
    RigidBodyDynamics::Math::SpatialVector world_Velocity_rHand;

    RigidBodyDynamics::Math::MatrixNd contactJacobian;

    RigidBodyDynamics::Math::MatrixNd zmpConsLambda;
    RigidBodyDynamics::Math::MatrixNd frictionConsLambda;
    RigidBodyDynamics::Math::MatrixNd unilateralConsLambda;

    // dimention
    int N;
    int n;
    int Nc;

    LowLevelController(ros::NodeHandle &nh);

    void updateControllerStates();

    RigidBodyDynamics::Math::VectorNd inverseDynamic(HighLevelCommand highLevelCommand);

    RigidBodyDynamics::Math::MatrixNd jointsSelectMatrix(int headIndex, int tailIndex);

    RigidBodyDynamics::Math::MatrixNd jointsSelectMatrix(std::string headJointName, std::string tailJointName);

    void homing();

    void homingSquat();


};

#endif //PROJECT_OPT_CONTROLLER_H
