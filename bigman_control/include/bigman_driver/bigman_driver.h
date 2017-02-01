#ifndef BIGMAN_DRIVER_H
#define BIGMAN_DRIVER_H


#include "ros/ros.h"
#include "ros/time.h"
#include <math.h>
#include <iostream>
#include <string.h>
#include <algorithm>
#include <Eigen/Dense>
#include <urdf/model.h>
#include "tf/transform_datatypes.h"
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <custom_effort_controllers/Command.h>
#include <custom_effort_controllers/CommandArrayStamped.h>
#include "utils.h"


class BigmanDriver {
public:

    // sub
    ros::Subscriber subJointStates;
    ros::Subscriber subModelStates;
    ros::Subscriber subLAnkleWrench;
    ros::Subscriber subRAnkleWrench;

    gazebo_msgs::ModelStates modelStates;
    sensor_msgs::JointState jointStates;
    geometry_msgs::WrenchStamped LAnkleWrench;
    geometry_msgs::WrenchStamped RAnkleWrench;

    // pub
    ros::Publisher pubJointCommands;


    // robot model information
    bool floating_base;
    bool whole_body;
    RigidBodyDynamics::Model model;  // model

    std::vector<std::string> joints; // RigidBodyDynamics order
    std::vector<std::string> bodies; // RigidBodyDynamics order
    std::vector<std::string> hardwareJoints; // group effort controller order

    int N; // total dof
    int n; // actuated dof

    // robot states information
    RigidBodyDynamics::Math::VectorNd q;  // general q
    RigidBodyDynamics::Math::VectorNd qd;  // general qd
    RigidBodyDynamics::Math::Vector3d pelvis_position;  // vector = [x,y,z]
    RigidBodyDynamics::Math::Vector3d pelvis_orientation_rpy; // euler_angle = [r, p, y]
    RigidBodyDynamics::Math::Vector4d pelvis_orientation_quaternion;  // quaternion = [x,y,z,w]
    RigidBodyDynamics::Math::Vector3d pelvis_linear_velocity;  // vector = [v_x,v_y,v_z]
    RigidBodyDynamics::Math::Vector3d pelvis_angular_velocity;  // vector = [omega_x, omega_y, omega_z]
    RigidBodyDynamics::Math::SpatialVector pelvis_spatial_velocity;

    RigidBodyDynamics::Math::SpatialVector lAnkle_spatial_force;
    RigidBodyDynamics::Math::SpatialVector rAnkle_spatial_force;


    // derived states
    RigidBodyDynamics::Math::Vector3d com;
    RigidBodyDynamics::Math::Vector3d com_velocity;
    RigidBodyDynamics::Math::Vector3d angular_momentum;
    double mass;
    double g;
    RigidBodyDynamics::Math::Vector3d G;

    // contact states
    double contactForceThreshold;
    std::string contactState;
    int contactNumber;


    BigmanDriver(ros::NodeHandle nh);

    void callbackJointStates(const sensor_msgs::JointState &msg);

    void callbackModelStates(const gazebo_msgs::ModelStates &msg);

    void callbackLAnkleWrench(const geometry_msgs::WrenchStamped &msg);

    void callbackRAnkleWrench(const geometry_msgs::WrenchStamped &msg);

    void updatePelvisState();

    void updateQ();

    void updateQd();

    void updateCoM();

    void updateContactState();

    void updateRobotStates();

    void printRobotStates();

    void publishCommand(RigidBodyDynamics::Math::VectorNd q, RigidBodyDynamics::Math::VectorNd tau);

    int getJointId(std::string name);

    int getBodyId(std::string name);

    RigidBodyDynamics::Math::VectorNd getNonlinearEffects();

    RigidBodyDynamics::Math::MatrixNd getInertiaMatrix();

    RigidBodyDynamics::Math::SpatialVector getBodyJdqd(std::string bodyName,
                                         RigidBodyDynamics::Math::Vector3d body_point = RigidBodyDynamics::Math::Vector3d::Zero());

    RigidBodyDynamics::Math::Vector3d getBodyPosition(std::string bodyName,
                                        RigidBodyDynamics::Math::Vector3d body_point = RigidBodyDynamics::Math::Vector3d::Zero());

    void getBodyJacobian(RigidBodyDynamics::Math::MatrixNd &J, std::string bodyName,
                         RigidBodyDynamics::Math::Vector3d body_point = RigidBodyDynamics::Math::Vector3d::Zero());





};


#endif //BIGMAN_DRIVER_H
