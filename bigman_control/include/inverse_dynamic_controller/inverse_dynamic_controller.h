#ifndef BIGMAN_CONTROL_INVERSE_DYNAMIC_CONTROLLER_H
#define BIGMAN_CONTROL_INVERSE_DYNAMIC_CONTROLLER_H

#include "ros/ros.h"
#include "ros/time.h"
#include <std_srvs/Empty.h>
#include "bigman_driver/bigman_driver.h"
#include "bigman_kinematic/bigman_kinematic.h"
#include "quadprog/quadprog.hpp"


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

    // constructor
    Task(std::string taskName, double weight, RigidBodyDynamics::Math::MatrixNd A, RigidBodyDynamics::Math::VectorNd b)
            : taskName(taskName), weight(weight), A(A), b(b) {}

    // function
    void disp() {
        std::cout << "---------------------------------------" << std::endl;
        std::cout << taskName << " with weight: " << weight << std::endl;
        std::cout << "A: " << "(" << A.rows() << "," << A.cols() << ")" << std::endl << A << std::endl;
        std::cout << "b.T: " << std::endl << b.transpose() << std::endl;
    }
};

class InverseDynamicController {

public:
    ros::NodeHandle nh_;

    // robot model information
    bool floating_base_;
    bool whole_body_;
    RigidBodyDynamics::Model model_;  // model

    std::vector<std::string> joints_; // RigidBodyDynamics order
    std::vector<std::string> bodies_; // RigidBodyDynamics order

    int N_; // total dof
    int n_; // actuated dof
    int Nc_; // contact numbers

    // robot states information
    RigidBodyDynamics::Math::VectorNd q_;  // general q
    RigidBodyDynamics::Math::VectorNd qd_;  // general qd

    // derived states
    RigidBodyDynamics::Math::Vector3d com_;
    RigidBodyDynamics::Math::Vector3d com_velocity_;
    RigidBodyDynamics::Math::Vector3d angular_momentum_;
    double mass_;
    double g_;
    RigidBodyDynamics::Math::Vector3d G_;

    // controller states
    RigidBodyDynamics::Math::MatrixNd inertiaMatrix_;
    RigidBodyDynamics::Math::VectorNd nonlinearEffects_;

    RigidBodyDynamics::Math::SpatialVector world_Acc_lSole_;
    RigidBodyDynamics::Math::SpatialVector world_Acc_rSole_;
    RigidBodyDynamics::Math::SpatialVector world_Acc_lHand_;
    RigidBodyDynamics::Math::SpatialVector world_Acc_rHand_;

    RigidBodyDynamics::Math::MatrixNd world_J_lSole_;
    RigidBodyDynamics::Math::MatrixNd world_J_rSole_;
    RigidBodyDynamics::Math::MatrixNd world_J_lHand_;
    RigidBodyDynamics::Math::MatrixNd world_J_rHand_;

    RigidBodyDynamics::Math::Vector3d world_Position_lSole_;
    RigidBodyDynamics::Math::Vector3d world_Position_rSole_;
    RigidBodyDynamics::Math::Vector3d world_Position_lHand_;
    RigidBodyDynamics::Math::Vector3d world_Position_rHand_;

    RigidBodyDynamics::Math::Matrix3d world_Rot_lSole_;
    RigidBodyDynamics::Math::Matrix3d world_Rot_rSole_;
    RigidBodyDynamics::Math::Matrix3d world_Rot_lHand_;
    RigidBodyDynamics::Math::Matrix3d world_Rot_rHand_;

    RigidBodyDynamics::Math::SpatialVector world_Velocity_lSole_;
    RigidBodyDynamics::Math::SpatialVector world_Velocity_rSole_;
    RigidBodyDynamics::Math::SpatialVector world_Velocity_lHand_;
    RigidBodyDynamics::Math::SpatialVector world_Velocity_rHand_;

    RigidBodyDynamics::Math::MatrixNd contactJacobian_;

    RigidBodyDynamics::Math::MatrixNd zmpConsLambda_;
    RigidBodyDynamics::Math::MatrixNd frictionConsLambda_;
    RigidBodyDynamics::Math::MatrixNd unilateralConsLambda_;


    // constructor
    InverseDynamicController(ros::NodeHandle &nh);

    void getRobotStates(const RigidBodyDynamics::Math::VectorNd &q, const RigidBodyDynamics::Math::VectorNd &qd);

    RigidBodyDynamics::Math::VectorNd CalcJointTorque(const HighLevelCommand &high_level_command);

    int getJointId(const std::string name);

    int getBodyId(const std::string name);

    RigidBodyDynamics::Math::VectorNd inverseDynamic(HighLevelCommand highLevelCommand);

    RigidBodyDynamics::Math::MatrixNd jointsSelectMatrix(int headIndex, int tailIndex);

    RigidBodyDynamics::Math::MatrixNd jointsSelectMatrix(std::string headJointName, std::string tailJointName);

    void homing();

    void homingSquat();


};

#endif //BIGMAN_CONTROL_INVERSE_DYNAMIC_CONTROLLER_H
