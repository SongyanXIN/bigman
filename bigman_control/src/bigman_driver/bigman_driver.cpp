#include "bigman_driver/bigman_driver.h"


BigmanDriver::BigmanDriver(ros::NodeHandle nh) {

    // load params from ros param server
    if (!nh.getParam("/floating_base", floating_base)) {
        ROS_ERROR("Failed to get param '/floating_base'");
    }
    if (!nh.getParam("/whole_body", whole_body)) {
        ROS_ERROR("Failed to get param '/whole_body'");
    }
    std::string urdf_string;
    if (!nh.getParam("/robot_description", urdf_string)) {
        ROS_ERROR("Failed to get param '/robot_description'");
    }
    RigidBodyDynamics::Addons::URDFReadFromString(urdf_string.c_str(), &model, floating_base);


    if (floating_base) {
        if (whole_body) {
            joints = {"tx", "ty", "tz", "rx", "ry", "rz",
                      "LHipLat", "LHipYaw", "LHipSag", "LKneeSag", "LAnkSag", "LAnkLat",
                      "RHipLat", "RHipYaw", "RHipSag", "RKneeSag", "RAnkSag", "RAnkLat",
                      "WaistLat", "WaistSag", "WaistYaw",
                      "LShSag", "LShLat", "LShYaw", "LElbj", "LForearmPlate", "LWrj1", "LWrj2",
                      "NeckYawj", "NeckPitchj",
                      "RShSag", "RShLat", "RShYaw", "RElbj", "RForearmPlate", "RWrj1", "RWrj2"};

            bodies = {"Translation_link", "Rotation_link", "base_link",
                      "LHipMot", "LThighUpLeg", "LThighLowLeg", "LLowLeg", "LFootmot", "LFoot",
                      "RHipMot", "RThighUpLeg", "RThighLowLeg", "RLowLeg", "RFootmot", "RFoot",
                      "DWL", "DWS", "DWYTorso",
                      "LShp", "LShr", "LShy", "LElb", "LForearm", "LWrMot2", "LWrMot3",
                      "NeckYaw", "NeckPitch",
                      "RShp", "RShr", "RShy", "RElb", "RForearm", "RWrMot2", "RWrMot3"};

        } else {
            joints = {"tx", "ty", "tz", "rx", "ry", "rz",
                      "LHipLat", "LHipYaw", "LHipSag", "LKneeSag", "LAnkSag", "LAnkLat",
                      "RHipLat", "RHipYaw", "RHipSag", "RKneeSag", "RAnkSag", "RAnkLat"};

            bodies = {"Translation_link", "Rotation_link", "base_link",
                      "LHipMot", "LThighUpLeg", "LThighLowLeg", "LLowLeg", "LFootmot", "LFoot",
                      "RHipMot", "RThighUpLeg", "RThighLowLeg", "RLowLeg", "RFootmot", "RFoot"};

        }
    } else {
        if (whole_body) {
            joints = {"LHipLat", "LHipYaw", "LHipSag", "LKneeSag", "LAnkSag", "LAnkLat",
                      "RHipLat", "RHipYaw", "RHipSag", "RKneeSag", "RAnkSag", "RAnkLat",
                      "WaistLat", "WaistSag", "WaistYaw",
                      "LShSag", "LShLat", "LShYaw", "LElbj", "LForearmPlate", "LWrj1", "LWrj2",
                      "NeckYawj", "NeckPitchj",
                      "RShSag", "RShLat", "RShYaw", "RElbj", "RForearmPlate", "RWrj1", "RWrj2"};

            bodies = {"base_link",
                      "LHipMot", "LThighUpLeg", "LThighLowLeg", "LLowLeg", "LFootmot", "LFoot",
                      "RHipMot", "RThighUpLeg", "RThighLowLeg", "RLowLeg", "RFootmot", "RFoot",
                      "DWL", "DWS", "DWYTorso",
                      "LShp", "LShr", "LShy", "LElb", "LForearm", "LWrMot2", "LWrMot3",
                      "NeckYaw", "NeckPitch",
                      "RShp", "RShr", "RShy", "RElb", "RForearm", "RWrMot2", "RWrMot3"};
        } else {
            joints = {"LHipLat", "LHipYaw", "LHipSag", "LKneeSag", "LAnkSag", "LAnkLat",
                      "RHipLat", "RHipYaw", "RHipSag", "RKneeSag", "RAnkSag", "RAnkLat"};

            bodies = {"base_link",
                      "LHipMot", "LThighUpLeg", "LThighLowLeg", "LLowLeg", "LFootmot", "LFoot",
                      "RHipMot", "RThighUpLeg", "RThighLowLeg", "RLowLeg", "RFootmot", "RFoot"};
        }
    }

    if (!nh.getParam("/bigman/group_position_torque_controller/joints", hardwareJoints)) {
        ROS_ERROR("Failed to get param '/bigman/group_position_torque_controller/joints'");
    }

    // subscriber
    subJointStates = nh.subscribe("/bigman/joint_states", 1, &BigmanDriver::callbackJointStates, this);
    subModelStates = nh.subscribe("/gazebo/model_states", 1, &BigmanDriver::callbackModelStates, this);
    subLAnkleWrench = nh.subscribe("/bigman/sensor/ft_sensor/LAnkle", 1, &BigmanDriver::callbackLAnkleWrench, this);
    subRAnkleWrench = nh.subscribe("/bigman/sensor/ft_sensor/RAnkle", 1, &BigmanDriver::callbackRAnkleWrench, this);

    while (jointStates.name.size() == 0 or modelStates.name.size() == 0
           or LAnkleWrench.header.seq == 0 or RAnkleWrench.header.seq == 0) {
        ros::spinOnce();
    }


    // publisher
    pubJointCommands = nh.advertise<custom_effort_controllers::CommandArrayStamped>(
            "/bigman/group_position_torque_controller/command", 1);

    // dof inf
    N = model.dof_count;
    if (floating_base) {
        n = N - 6;
    } else {
        n = N;
    }



    // basic states
    q = RigidBodyDynamics::Math::VectorNd::Zero(N);
    qd = RigidBodyDynamics::Math::VectorNd::Zero(N);
    pelvis_position = RigidBodyDynamics::Math::Vector3d::Zero();  // vector = [x,y,z]
    pelvis_orientation_rpy = RigidBodyDynamics::Math::Vector3d::Zero(); // euler_angle = [r, p, y]
    pelvis_orientation_quaternion = RigidBodyDynamics::Math::Vector4d::Zero();   // quaternion = [x,y,z,w]
    pelvis_linear_velocity = RigidBodyDynamics::Math::Vector3d::Zero();   // vector = [v_x,v_y,v_z]
    pelvis_angular_velocity = RigidBodyDynamics::Math::Vector3d::Zero();  // vector = [omega_x, omega_y, omega_z]

    com = RigidBodyDynamics::Math::Vector3d::Zero();
    com_velocity = RigidBodyDynamics::Math::Vector3d::Zero();
    angular_momentum = RigidBodyDynamics::Math::Vector3d::Zero();
    mass = 0;
    g = -9.81;
    G = RigidBodyDynamics::Math::Vector3d(0.0, 0.0, -9.81);

    contactForceThreshold = 100;
    contactState = "noSupport";
    contactNumber = 0;

}


void BigmanDriver::callbackJointStates(const sensor_msgs::JointState &msg) {

    jointStates = msg;

}

void BigmanDriver::callbackModelStates(const gazebo_msgs::ModelStates &msg) {

    modelStates = msg;

}

void BigmanDriver::callbackLAnkleWrench(const geometry_msgs::WrenchStamped &msg) {

    LAnkleWrench = msg;
}

void BigmanDriver::callbackRAnkleWrench(const geometry_msgs::WrenchStamped &msg) {

    RAnkleWrench = msg;
}

void BigmanDriver::updatePelvisState() {


    pelvis_position[0] = modelStates.pose[1].position.x;
    pelvis_position[1] = modelStates.pose[1].position.y;
    pelvis_position[2] = modelStates.pose[1].position.z;

    pelvis_orientation_quaternion[0] = modelStates.pose[1].orientation.x;
    pelvis_orientation_quaternion[1] = modelStates.pose[1].orientation.y;
    pelvis_orientation_quaternion[2] = modelStates.pose[1].orientation.z;
    pelvis_orientation_quaternion[3] = modelStates.pose[1].orientation.w;

    tf::Quaternion tfQuaternion;
    tf::quaternionMsgToTF(modelStates.pose[1].orientation, tfQuaternion);
    tf::Matrix3x3(tfQuaternion).getRPY(pelvis_orientation_rpy[0], pelvis_orientation_rpy[1],
                                       pelvis_orientation_rpy[2]);

    pelvis_linear_velocity[0] = modelStates.twist[1].linear.x;
    pelvis_linear_velocity[1] = modelStates.twist[1].linear.y;
    pelvis_linear_velocity[2] = modelStates.twist[1].linear.z;

    pelvis_angular_velocity[0] = modelStates.twist[1].angular.x;
    pelvis_angular_velocity[1] = modelStates.twist[1].angular.y;
    pelvis_angular_velocity[2] = modelStates.twist[1].angular.z;

    pelvis_spatial_velocity[0] = modelStates.twist[1].angular.x;
    pelvis_spatial_velocity[1] = modelStates.twist[1].angular.y;
    pelvis_spatial_velocity[2] = modelStates.twist[1].angular.z;
    pelvis_spatial_velocity[3] = modelStates.twist[1].linear.x;
    pelvis_spatial_velocity[4] = modelStates.twist[1].linear.y;
    pelvis_spatial_velocity[5] = modelStates.twist[1].linear.z;

}

void BigmanDriver::updateQ() {

    if (floating_base) {
        q[0] = pelvis_position[0];
        q[1] = pelvis_position[1];
        q[2] = pelvis_position[2];

        q[3] = pelvis_orientation_rpy[0];
        q[4] = pelvis_orientation_rpy[1];
        q[5] = pelvis_orientation_rpy[2];


        std::map<std::string, double> mapJointNamePosition;
        for (int i = 0; i < jointStates.name.size(); ++i) {
            mapJointNamePosition.insert(std::pair<std::string, double>(jointStates.name[i], jointStates.position[i]));
        }

        for (int i = 6; i < N; ++i) {
            q(i) = mapJointNamePosition[joints[i]];
        }
    } else {

        std::map<std::string, double> mapJointNamePosition;
        for (int i = 0; i < jointStates.name.size(); ++i) {
            mapJointNamePosition.insert(std::pair<std::string, double>(jointStates.name[i], jointStates.position[i]));
        }

        for (int i = 0; i < N; ++i) {
            q(i) = mapJointNamePosition[joints[i]];
        }
    }


}

void BigmanDriver::updateQd() {
    if (floating_base) {
        RigidBodyDynamics::Math::MatrixNd world_J_Pelvis = RigidBodyDynamics::Math::MatrixNd::Zero(6, N);
        CalcPointJacobian6D(model,
                            q,
                            getBodyId("base_link"),
                            RigidBodyDynamics::Math::Vector3d::Zero(),
                            world_J_Pelvis);

        qd.head(6) << world_J_Pelvis.block(0, 0, 6, 6).inverse() * pelvis_spatial_velocity;

        std::map<std::string, double> mapJointNameVelocity;
        for (int i = 0; i < jointStates.name.size(); ++i) {
            mapJointNameVelocity.insert(std::pair<std::string, double>(jointStates.name[i], jointStates.velocity[i]));
        }

        for (int i = 6; i < N; ++i) {
            qd(i) = mapJointNameVelocity[joints[i]];
        }
    } else {
        std::map<std::string, double> mapJointNameVelocity;
        for (int i = 0; i < jointStates.name.size(); ++i) {
            mapJointNameVelocity.insert(std::pair<std::string, double>(jointStates.name[i], jointStates.velocity[i]));
        }

        for (int i = 0; i < N; ++i) {
            qd(i) = mapJointNameVelocity[joints[i]];
        }
    }


}

void BigmanDriver::updateCoM() {
    RigidBodyDynamics::Utils::CalcCenterOfMass(model, q, qd, mass, com, &com_velocity, &angular_momentum);
}

void BigmanDriver::updateContactState() {

    lAnkle_spatial_force[0] = LAnkleWrench.wrench.torque.x;
    lAnkle_spatial_force[1] = LAnkleWrench.wrench.torque.y;
    lAnkle_spatial_force[2] = LAnkleWrench.wrench.torque.z;
    lAnkle_spatial_force[3] = LAnkleWrench.wrench.force.x;
    lAnkle_spatial_force[4] = LAnkleWrench.wrench.force.y;
    lAnkle_spatial_force[5] = LAnkleWrench.wrench.force.z;

    rAnkle_spatial_force[0] = RAnkleWrench.wrench.torque.x;
    rAnkle_spatial_force[1] = RAnkleWrench.wrench.torque.y;
    rAnkle_spatial_force[2] = RAnkleWrench.wrench.torque.z;
    rAnkle_spatial_force[3] = RAnkleWrench.wrench.force.x;
    rAnkle_spatial_force[4] = RAnkleWrench.wrench.force.y;
    rAnkle_spatial_force[5] = RAnkleWrench.wrench.force.z;

    if (std::abs(rAnkle_spatial_force[5]) > contactForceThreshold and
        std::abs(lAnkle_spatial_force[5]) > contactForceThreshold) {
        contactState = "doubleSupport";
        contactNumber = 2;
    } else if (std::abs(rAnkle_spatial_force[5]) <= contactForceThreshold and
               std::abs(lAnkle_spatial_force[5]) <= contactForceThreshold) {
        contactState = "noSupport";
        contactNumber = 0;
    } else if (std::abs(rAnkle_spatial_force[5]) <= contactForceThreshold and
               contactForceThreshold < std::abs(lAnkle_spatial_force[5])) {
        contactState = "leftSupport";
        contactNumber = 1;
    } else if (std::abs(rAnkle_spatial_force[5]) > contactForceThreshold and
               contactForceThreshold >= std::abs(lAnkle_spatial_force[5])) {
        contactState = "rightSupport";
        contactNumber = 1;
    }


}

void BigmanDriver::updateRobotStates() {

    updatePelvisState();
    updateQ();
    updateQd();
    updateCoM();
    updateContactState();

}

void BigmanDriver::printRobotStates() {

    std::string sep = "\n---------------------------------------------------------------\n";
    std::cout << sep;
    std::cout << "Floating_base: " << floating_base << sep;
    std::cout << "Whole_body: " << whole_body << sep;
    std::cout << "Robot Dof: " << N << ", Actuated: " << n << sep;
    std::cout << "Pelvis position   : " << pelvis_position.transpose().format(CleanFmt) << sep;
    std::cout << "Pelvis orientation: " << pelvis_orientation_rpy.transpose().format(CleanFmt) << sep;
    std::cout << "Pelvis linear vel : " << pelvis_linear_velocity.transpose().format(CleanFmt) << sep;
    std::cout << "Pelvis angular vel: " << pelvis_angular_velocity.transpose().format(CleanFmt) << sep;
    std::cout << "Q:" << q.transpose().format(CleanFmt) << sep;
    std::cout << "Qd:" << qd.transpose().format(CleanFmt) << sep;
    std::cout << sep;
}


void BigmanDriver::publishCommand(RigidBodyDynamics::Math::VectorNd q, RigidBodyDynamics::Math::VectorNd tau) {
    // input q and tau should be of length N, this publish function will only publish the actuated joint command.
    // check q and tau dimension
    if (q.size() != tau.size()){
        std::cerr << "Error: BigmanDriver::publishCommand(), q and tau should have the same length!" << std::endl;
    }
    else {
            if (floating_base && (q.size() == N)) { // N > n
                    auto cmdArrayStamped = custom_effort_controllers::CommandArrayStamped();
                    for (int i = 0; i < n; ++i) {
                        auto cmd = custom_effort_controllers::Command();
                        cmd.position = q[i + 6];
                        cmd.torque = tau[i + 6];
                        cmdArrayStamped.commands.push_back(cmd);
                    }
                    pubJointCommands.publish(cmdArrayStamped);
            }
            else {
                auto cmdArrayStamped = custom_effort_controllers::CommandArrayStamped();
                for (int i = 0; i < n; ++i) {
                    auto cmd = custom_effort_controllers::Command();
                    cmd.position = q[i];
                    cmd.torque = tau[i];
                    cmdArrayStamped.commands.push_back(cmd);
                }
                pubJointCommands.publish(cmdArrayStamped);
            }

    }

}

int BigmanDriver::getBodyId(std::string name) {
    std::vector<std::string>::iterator it = std::find(bodies.begin(), bodies.end(), name);
    if (it == bodies.end()) {
        std::cerr << "Error: Body Id not found! Please check body name!" << std::endl;
    } else {
        return it - bodies.begin();
    }

}

int BigmanDriver::getJointId(std::string name) {
    std::vector<std::string>::iterator it = std::find(joints.begin(), joints.end(), name);
    if (it == joints.end()) {
        std::cerr << "Error: Joint Id not found! Please check joint name!" << std::endl;
    } else {
        return it - joints.begin();
    }

}

RigidBodyDynamics::Math::VectorNd BigmanDriver::getNonlinearEffects() {
    RigidBodyDynamics::Math::VectorNd tau = RigidBodyDynamics::Math::VectorNd::Zero(n);
    RigidBodyDynamics::NonlinearEffects(model, q, qd, tau);
    return tau;
}

RigidBodyDynamics::Math::MatrixNd BigmanDriver::getInertiaMatrix() {
    RigidBodyDynamics::Math::MatrixNd H = RigidBodyDynamics::Math::MatrixNd::Zero(N, N);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model, q, H);
    return H;
}

RigidBodyDynamics::Math::SpatialVector
BigmanDriver::getBodyJdqd(std::string bodyName, RigidBodyDynamics::Math::Vector3d body_point) {
    return RigidBodyDynamics::CalcPointAcceleration6D(model,
                                                      q,
                                                      qd,
                                                      RigidBodyDynamics::Math::VectorNd::Zero(N),
                                                      getBodyId(bodyName),
                                                      body_point);
}

RigidBodyDynamics::Math::Vector3d
BigmanDriver::getBodyPosition(std::string bodyName, RigidBodyDynamics::Math::Vector3d body_point) {
    return RigidBodyDynamics::CalcBodyToBaseCoordinates(model,
                                                        q,
                                                        getBodyId(bodyName),
                                                        body_point);
}

void BigmanDriver::getBodyJacobian(RigidBodyDynamics::Math::MatrixNd &J, std::string bodyName,
                                   RigidBodyDynamics::Math::Vector3d body_point) {
    RigidBodyDynamics::CalcPointJacobian6D(model,
                                           q,
                                           getBodyId(bodyName),
                                           body_point,
                                           J);

}







