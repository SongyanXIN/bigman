#include "inverse_dynamic_controller/inverse_dynamic_controller.h"



InverseDynamicController::InverseDynamicController(ros::NodeHandle &nh) : nh_(nh){

    // load params from ros param server
    if (!nh.getParam("/floating_base", floating_base_)) {
        ROS_ERROR("Failed to get param '/floating_base'");
    }
    if (!nh.getParam("/whole_body", whole_body_)) {
        ROS_ERROR("Failed to get param '/whole_body'");
    }
    std::string urdf_string;
    if (!nh.getParam("/robot_description", urdf_string)) {
        ROS_ERROR("Failed to get param '/robot_description'");
    }
    RigidBodyDynamics::Addons::URDFReadFromString(urdf_string.c_str(), &model_, floating_base_);


    if (floating_base_) {
        if (whole_body_) {
            joints_ = {"tx", "ty", "tz", "rx", "ry", "rz",
                      "LHipLat", "LHipYaw", "LHipSag", "LKneeSag", "LAnkSag", "LAnkLat",
                      "RHipLat", "RHipYaw", "RHipSag", "RKneeSag", "RAnkSag", "RAnkLat",
                      "WaistLat", "WaistSag", "WaistYaw",
                      "LShSag", "LShLat", "LShYaw", "LElbj", "LForearmPlate", "LWrj1", "LWrj2",
                      "NeckYawj", "NeckPitchj",
                      "RShSag", "RShLat", "RShYaw", "RElbj", "RForearmPlate", "RWrj1", "RWrj2"};

            bodies_ = {"Translation_link", "Rotation_link", "base_link",
                      "LHipMot", "LThighUpLeg", "LThighLowLeg", "LLowLeg", "LFootmot", "LFoot",
                      "RHipMot", "RThighUpLeg", "RThighLowLeg", "RLowLeg", "RFootmot", "RFoot",
                      "DWL", "DWS", "DWYTorso",
                      "LShp", "LShr", "LShy", "LElb", "LForearm", "LWrMot2", "LWrMot3",
                      "NeckYaw", "NeckPitch",
                      "RShp", "RShr", "RShy", "RElb", "RForearm", "RWrMot2", "RWrMot3"};

        } else {
            joints_ = {"tx", "ty", "tz", "rx", "ry", "rz",
                      "LHipLat", "LHipYaw", "LHipSag", "LKneeSag", "LAnkSag", "LAnkLat",
                      "RHipLat", "RHipYaw", "RHipSag", "RKneeSag", "RAnkSag", "RAnkLat"};

            bodies_ = {"Translation_link", "Rotation_link", "base_link",
                      "LHipMot", "LThighUpLeg", "LThighLowLeg", "LLowLeg", "LFootmot", "LFoot",
                      "RHipMot", "RThighUpLeg", "RThighLowLeg", "RLowLeg", "RFootmot", "RFoot"};

        }
    } else {
        if (whole_body_) {
            joints_ = {"LHipLat", "LHipYaw", "LHipSag", "LKneeSag", "LAnkSag", "LAnkLat",
                      "RHipLat", "RHipYaw", "RHipSag", "RKneeSag", "RAnkSag", "RAnkLat",
                      "WaistLat", "WaistSag", "WaistYaw",
                      "LShSag", "LShLat", "LShYaw", "LElbj", "LForearmPlate", "LWrj1", "LWrj2",
                      "NeckYawj", "NeckPitchj",
                      "RShSag", "RShLat", "RShYaw", "RElbj", "RForearmPlate", "RWrj1", "RWrj2"};

            bodies_ = {"base_link",
                      "LHipMot", "LThighUpLeg", "LThighLowLeg", "LLowLeg", "LFootmot", "LFoot",
                      "RHipMot", "RThighUpLeg", "RThighLowLeg", "RLowLeg", "RFootmot", "RFoot",
                      "DWL", "DWS", "DWYTorso",
                      "LShp", "LShr", "LShy", "LElb", "LForearm", "LWrMot2", "LWrMot3",
                      "NeckYaw", "NeckPitch",
                      "RShp", "RShr", "RShy", "RElb", "RForearm", "RWrMot2", "RWrMot3"};
        } else {
            joints_ = {"LHipLat", "LHipYaw", "LHipSag", "LKneeSag", "LAnkSag", "LAnkLat",
                      "RHipLat", "RHipYaw", "RHipSag", "RKneeSag", "RAnkSag", "RAnkLat"};

            bodies_ = {"base_link",
                      "LHipMot", "LThighUpLeg", "LThighLowLeg", "LLowLeg", "LFootmot", "LFoot",
                      "RHipMot", "RThighUpLeg", "RThighLowLeg", "RLowLeg", "RFootmot", "RFoot"};
        }
    }

    N_ = model_.dof_count;
    if (floating_base_) {
        n_ = N_ - 6;
    } else {
        n_ = N_;
    }
    Nc_ = 2; // default: double support

    q_ = RigidBodyDynamics::Math::VectorNd::Zero(N_);
    qd_ = RigidBodyDynamics::Math::VectorNd::Zero(N_);

    com_ = RigidBodyDynamics::Math::Vector3d::Zero();
    com_velocity_ = RigidBodyDynamics::Math::Vector3d::Zero();
    angular_momentum_ = RigidBodyDynamics::Math::Vector3d::Zero();
    mass_ = 0;
    g_ = -9.81;
    G_ = RigidBodyDynamics::Math::Vector3d(0.0, 0.0, -9.81);

    inertiaMatrix_ = RigidBodyDynamics::Math::MatrixNd::Zero(N_, N_);
    nonlinearEffects_ = RigidBodyDynamics::Math::VectorNd::Zero(N_);

    world_Acc_lSole_ = RigidBodyDynamics::Math::SpatialVector::Zero();
    world_Acc_rSole_ = RigidBodyDynamics::Math::SpatialVector::Zero();
    world_Acc_lHand_ = RigidBodyDynamics::Math::SpatialVector::Zero();
    world_Acc_rHand_ = RigidBodyDynamics::Math::SpatialVector::Zero();

    world_J_lSole_ = RigidBodyDynamics::Math::MatrixNd::Zero(6, N_);
    world_J_rSole_ = RigidBodyDynamics::Math::MatrixNd::Zero(6, N_);
    world_J_lHand_ = RigidBodyDynamics::Math::MatrixNd::Zero(6, N_);
    world_J_rHand_ = RigidBodyDynamics::Math::MatrixNd::Zero(6, N_);

    world_Position_lSole_ = RigidBodyDynamics::Math::Vector3d::Zero();
    world_Position_rSole_ = RigidBodyDynamics::Math::Vector3d::Zero();
    world_Position_lHand_ = RigidBodyDynamics::Math::Vector3d::Zero();
    world_Position_rHand_ = RigidBodyDynamics::Math::Vector3d::Zero();

    world_Rot_lSole_ = RigidBodyDynamics::Math::Matrix3d::Zero();
    world_Rot_rSole_ = RigidBodyDynamics::Math::Matrix3d::Zero();
    world_Rot_lHand_ = RigidBodyDynamics::Math::Matrix3d::Zero();
    world_Rot_rHand_ = RigidBodyDynamics::Math::Matrix3d::Zero();

    world_Velocity_lSole_ = RigidBodyDynamics::Math::SpatialVector::Zero();
    world_Velocity_rSole_ = RigidBodyDynamics::Math::SpatialVector::Zero();
    world_Velocity_lHand_ = RigidBodyDynamics::Math::SpatialVector::Zero();
    world_Velocity_rHand_ = RigidBodyDynamics::Math::SpatialVector::Zero();

    contactJacobian_ = RigidBodyDynamics::Math::MatrixNd::Zero(12, N_); // default: double support

    // inequality constraints
    // zmp constraints
    double dx_min = -0.15;
    double dx_max = 0.15;
    double dy_min = -0.07;
    double dy_max = 0.07;
    RigidBodyDynamics::Math::MatrixNd zmpConsSpatialForce = RigidBodyDynamics::Math::MatrixNd::Zero(4, 6);
    zmpConsSpatialForce << -1, 0, 0, 0, 0, dy_min,
            1, 0, 0, 0, 0, -dy_max,
            0, 1, 0, 0, 0, dx_min,
            0, -1, 0, 0, 0, -dx_max;

    zmpConsLambda_ = RigidBodyDynamics::Math::MatrixNd::Zero(8, 12);
    zmpConsLambda_ << zmpConsSpatialForce, RigidBodyDynamics::Math::MatrixNd::Zero(4, 6),
            RigidBodyDynamics::Math::MatrixNd::Zero(4, 6), zmpConsSpatialForce;


    // friction constraints
    // friction constraints matrix for spatial force [M^T,F^T]^T = [Mx,My,Mz,Fx,Fy,Fz]^T
    double mu = 0.7;
    RigidBodyDynamics::Math::MatrixNd frictionConsSpatialForce = RigidBodyDynamics::Math::MatrixNd::Zero(4, 6);
    frictionConsSpatialForce << 0, 0, 0, 1, 0, -mu,
            0, 0, 0, -1, 0, -mu,
            0, 0, 0, 0, 1, -mu,
            0, 0, 0, 0, -1, -mu;
    frictionConsLambda_ = RigidBodyDynamics::Math::MatrixNd::Zero(8, 12);
    frictionConsLambda_ << frictionConsSpatialForce, RigidBodyDynamics::Math::MatrixNd::Zero(4, 6),
            RigidBodyDynamics::Math::MatrixNd::Zero(4, 6), frictionConsSpatialForce;

    // unilateral constraints
    RigidBodyDynamics::Math::MatrixNd unilateralConsSpatialForce = RigidBodyDynamics::Math::MatrixNd::Zero(1, 6);
    unilateralConsSpatialForce << 0, 0, 0, 0, 0, -1;
    unilateralConsLambda_ = RigidBodyDynamics::Math::MatrixNd::Zero(2, 12);
    unilateralConsLambda_ << unilateralConsSpatialForce, RigidBodyDynamics::Math::MatrixNd::Zero(1, 6),
            RigidBodyDynamics::Math::MatrixNd::Zero(1, 6), unilateralConsSpatialForce;


}



void InverseDynamicController::getRobotStates(const RigidBodyDynamics::Math::VectorNd &q, const RigidBodyDynamics::Math::VectorNd &qd){
    q_ = q;
    qd_ = qd;
    RigidBodyDynamics::Utils::CalcCenterOfMass(model_, q_, qd_, mass_, com_, &com_velocity_, &angular_momentum_);

    inertiaMatrix_.setZero();
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, q_, inertiaMatrix_);

    nonlinearEffects_.setZero();
    RigidBodyDynamics::NonlinearEffects(model_, q_, qd_, nonlinearEffects_);

    world_Acc_lSole_ = RigidBodyDynamics::CalcPointAcceleration6D(model_,
                                                                 q_,
                                                                 qd_,
                                                                 RigidBodyDynamics::Math::VectorNd::Zero(N_),
                                                                 getBodyId("LFoot"),
                                                                 RigidBodyDynamics::Math::Vector3d(0, 0, -0.1435));
    world_Acc_rSole_ = RigidBodyDynamics::CalcPointAcceleration6D(model_,
                                                                 q_,
                                                                 qd_,
                                                                 RigidBodyDynamics::Math::VectorNd::Zero(N_),
                                                                 getBodyId("RFoot"),
                                                                 RigidBodyDynamics::Math::Vector3d(0, 0, -0.1435));

    world_Acc_lHand_ = RigidBodyDynamics::CalcPointAcceleration6D(model_,
                                                                 q_,
                                                                 qd_,
                                                                 RigidBodyDynamics::Math::VectorNd::Zero(N_),
                                                                 getBodyId("LWrMot3"),
                                                                 RigidBodyDynamics::Math::Vector3d::Zero());
    world_Acc_rHand_ = RigidBodyDynamics::CalcPointAcceleration6D(model_,
                                                                 q_,
                                                                 qd_,
                                                                 RigidBodyDynamics::Math::VectorNd::Zero(N_),
                                                                 getBodyId("RWrMot3"),
                                                                 RigidBodyDynamics::Math::Vector3d::Zero());


    world_J_lSole_.setZero();
    world_J_rSole_.setZero();
    world_J_lHand_.setZero();
    world_J_rHand_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_,
                                           q_,
                                           getBodyId("LFoot"),
                                           RigidBodyDynamics::Math::Vector3d(0, 0, -0.1435),
                                           world_J_lSole_);
    RigidBodyDynamics::CalcPointJacobian6D(model_,
                                           q_,
                                           getBodyId("RFoot"),
                                           RigidBodyDynamics::Math::Vector3d(0, 0, -0.1435),
                                           world_J_rSole_);
    RigidBodyDynamics::CalcPointJacobian6D(model_,
                                           q_,
                                           getBodyId("LWrMot3"),
                                           RigidBodyDynamics::Math::Vector3d::Zero(),
                                           world_J_lHand_);
    RigidBodyDynamics::CalcPointJacobian6D(model_,
                                           q_,
                                           getBodyId("RWrMot3"),
                                           RigidBodyDynamics::Math::Vector3d::Zero(),
                                           world_J_rHand_);


    world_Position_lSole_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,
                                                                        q_,
                                                                        getBodyId("LFoot"),
                                                                        RigidBodyDynamics::Math::Vector3d(0, 0, -0.1435));
    world_Position_rSole_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,
                                                                        q_,
                                                                        getBodyId("RFoot"),
                                                                        RigidBodyDynamics::Math::Vector3d(0, 0, -0.1435));
    world_Position_lHand_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,
                                                                        q_,
                                                                        getBodyId("LWrMot3"),
                                                                        RigidBodyDynamics::Math::Vector3d::Zero());
    world_Position_rHand_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,
                                                                        q_,
                                                                        getBodyId("RWrMot3"),
                                                                        RigidBodyDynamics::Math::Vector3d::Zero());


    world_Rot_lSole_ = RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_, getBodyId("LFoot")).transpose();
    world_Rot_rSole_ = RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_, getBodyId("RFoot")).transpose();
    world_Rot_lHand_ = RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_,
                                                                  getBodyId("LWrMot3")).transpose();
    world_Rot_rHand_ = RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_,
                                                                  getBodyId("RWrMot3")).transpose();


    world_Velocity_lSole_ = RigidBodyDynamics::CalcPointVelocity6D(model_,
                                                                  q_,
                                                                  qd_,
                                                                  getBodyId("LFoot"),
                                                                  RigidBodyDynamics::Math::Vector3d(0, 0, -0.1435));
    world_Velocity_rSole_ = RigidBodyDynamics::CalcPointVelocity6D(model_,
                                                                  q_,
                                                                  qd_,
                                                                  getBodyId("RFoot"),
                                                                  RigidBodyDynamics::Math::Vector3d(0, 0, -0.1435));
    world_Velocity_lHand_ = RigidBodyDynamics::CalcPointVelocity6D(model_,
                                                                  q_,
                                                                  qd_,
                                                                  getBodyId("LWrMot3"),
                                                                  RigidBodyDynamics::Math::Vector3d::Zero());
    world_Velocity_rHand_ = RigidBodyDynamics::CalcPointVelocity6D(model_,
                                                                  q_,
                                                                  qd_,
                                                                  getBodyId("RWrMot3"),
                                                                  RigidBodyDynamics::Math::Vector3d::Zero());

}

RigidBodyDynamics::Math::VectorNd InverseDynamicController::CalcJointTorque(const HighLevelCommand &highLevelCommand) {
    using namespace std;
    using namespace RigidBodyDynamics;
    using namespace RigidBodyDynamics::Math;



    // overwrite with commanded contact state
    switch (highLevelCommand.contactState) {
        case noSupport: {
            Nc_ = 0;
            break;
        }
        case leftSupport: {
            Nc_ = 1;
            contactJacobian_.resize(6, N_);
            contactJacobian_ << world_J_lSole_;
            break;
        }
        case rightSupport: {
            Nc_ = 1;
            contactJacobian_.resize(6, N_);
            contactJacobian_ << world_J_rSole_;
            break;
        }
        case doubleSupport: {
            Nc_ = 2;
            contactJacobian_.resize(12, N_);
            contactJacobian_ << world_J_rSole_, world_J_lSole_;
            break;
        }
    }



    // container for all tasks
    std::vector<Task> tasks = {};

    // least square task
    MatrixNd ls_A = MatrixNd::Identity(N_ + 6 * Nc_, N_ + 6 * Nc_);
    VectorNd ls_b = VectorNd::Zero(N_ + 6 * Nc_);
    tasks.push_back(Task("least square task", 1, ls_A, ls_b));

    // CoM->GRF task
    Vector3d desired_com_acc = Vector3d::Zero();
    desired_com_acc = 10 * (highLevelCommand.CoMPosRef - com_) - 1 * (com_velocity_);
    Vector3d desired_Ld = Vector3d::Zero();
    desired_Ld = -3 * (angular_momentum_);

    Vector3d pr = Vector3d::Zero();
    Vector3d pl = Vector3d::Zero();
    pr = world_Position_rSole_ - com_;
    pl = world_Position_lSole_ - com_;

    Matrix3d Prx = Matrix3d::Zero();
    Matrix3d Plx = Matrix3d::Zero();
    Prx = skew(pr);
    Plx = skew(pl);

    MatrixNd Tr = MatrixNd::Zero(6, 6);
    MatrixNd Tl = MatrixNd::Zero(6, 6);
    Tr << Matrix3d::Zero(), Matrix3d::Identity(), Matrix3d::Identity(), Prx;
    Tl << Matrix3d::Zero(), Matrix3d::Identity(), Matrix3d::Identity(), Plx;

    MatrixNd com_A = MatrixNd::Zero(6, N_ + 6 * Nc_);
    VectorNd com_b = VectorNd::Zero(6);

    com_A << MatrixNd::Zero(6, N_), Tr, Tl;
    com_b << mass_ * (desired_com_acc - G_), desired_Ld;

    tasks.push_back(Task("GRF task", 100, com_A, com_b));

    // GRF distribution
    switch (highLevelCommand.contactState) {
        case noSupport: {
            break;
        }
        case leftSupport: {
            break;
        }
        case rightSupport: {
            break;
        }
        case doubleSupport: {
            double dr = (world_Position_rSole_.head(2) - com_.head(2)).norm();
            double dl = (world_Position_lSole_.head(2) - com_.head(2)).norm();
            double wr = dl / (dr + dl);
            double wl = dr / (dr + dl);
            MatrixNd GRF_dis_A = MatrixNd::Zero(6, N_ + 6 * Nc_);
            VectorNd GRF_dis_b = VectorNd::Zero(6);
            GRF_dis_A << MatrixNd::Zero(6, N_), wl * MatrixNd::Identity(6, 6), -wr * MatrixNd::Identity(6, 6);
            tasks.push_back(Task("GRF distribution task", 100, GRF_dis_A, GRF_dis_b));
            break;
        }


    }



    // pelvis position

    // pelvis orientation
    MatrixNd pelvis_orientation_A = MatrixNd::Zero(3, N_ + 6 * Nc_);
    VectorNd pelvis_orientation_b = VectorNd::Zero(3);
    pelvis_orientation_A << jointsSelectMatrix("rx", "rz");
    VectorNd pelvis_orientation_des = VectorNd::Zero(3);
    pelvis_orientation_b << 500 * (pelvis_orientation_des - q_.segment(3, 3)) - 10 * qd_.segment(3, 3);

    tasks.push_back(Task("pelvis orientation task", 100, pelvis_orientation_A, pelvis_orientation_b));

//        cout << "pelvis_orientation_b: " << pelvis_orientation_b.transpose() << endl;


//        // pelvis task
//        MatrixNd pelvis_A = MatrixNd::Zero(6, N_ + 6 * Nc_);
//        VectorNd pelvis_b = VectorNd::Zero(6);
//        pelvis_A << MatrixNd::Identity(6, 6), MatrixNd::Zero(6, N_ + 6 * Nc_ - 6);
//        VectorNd pelvis_des = VectorNd::Zero(6);
//        pelvis_des << q_.head(3), 0, 0, 0;
//        pelvis_b << 500 * (pelvis_des - q_.head(6)) - 10 * qd_.head(6);
//        cout << "pelvis_b: " << pelvis_b.transpose() << endl;
//
//        tasks.push_back(Task("pelvis task", 100, pelvis_A, pelvis_b));


    // upperbody task
    MatrixNd upperbody_A = MatrixNd::Zero(19, N_ + 6 * Nc_);
    VectorNd upperbody_b = VectorNd::Zero(19);
    upperbody_A << MatrixNd::Zero(19, 18), MatrixNd::Identity(19, 19), MatrixNd::Zero(19, 12);
    VectorNd upperbodyConfigDes = VectorNd::Zero(19);
    //upperbodyConfigDes(6) = deg2rad(-45);
    //upperbodyConfigDes(15) = deg2rad(-45);
    upperbody_b << 100 * (upperbodyConfigDes - q_.segment(18, 19)) - 10 * qd_.segment(18, 19);
    tasks.push_back(Task("upperbody task", 70, upperbody_A, upperbody_b));



//        // head task
//        MatrixNd head_A = jointsSelectMatrix("NeckYawj", "NeckPitchj");
//        VectorNd head_b = VectorNd::Zero(2);
//        VectorNd headQDes = VectorNd::Zero(2);
//        headQDes << deg2rad(30), deg2rad(-0);
//        head_b << jointsPD(q_.segment(driver.getJointId("NeckYawj"), 2),
//                           qd_.segment(driver.getJointId("NeckPitchj"), 2),
//                           headQDes);
//        tasks.push_back(Task("head task", 30, head_A, head_b));






//        // left hand tracking task
//        MatrixNd LHandTracking_A = MatrixNd::Zero(6, N_ + 6 * Nc_);
//        VectorNd LHandTracking_b = VectorNd::Zero(6);
//        LHandTracking_A << world_J_lHand, MatrixNd::Zero(6, 6 * Nc_);
//        SpatialVector cmdAccSpatialLHand = cartesianPD(world_Position_lHand, world_Rot_lHand,
//                                                  world_Velocity_lHand.tail(3), world_Velocity_lHand.head(3),
//                                                  Vector3d(0.12, 0.5, 0.8), Matrix3d::Identity());
//        LHandTracking_b = cmdAccSpatialLHand - world_Acc_lHand;
//        tasks.push_back(Task("left foot tracking task", 50, LHandTracking_A, LHandTracking_b));
//
//
//
//
//
//        // right hand tracking task
//        MatrixNd RHandTracking_A = MatrixNd::Zero(6, N_ + 6 * Nc_);
//        VectorNd RHandTracking_b = VectorNd::Zero(6);
//        RHandTracking_A << world_J_rHand, MatrixNd::Zero(6, 6 * Nc_);
//        SpatialVector cmdAccSpatialRHand = cartesianPD(world_Position_rHand, world_Rot_rHand,
//                                                  world_Velocity_rHand.tail(3), world_Velocity_rHand.head(3),
//                                                  Vector3d(0.12, -0.5, 0.8), Matrix3d::Identity());
//
//        RHandTracking_b = cmdAccSpatialRHand - world_Acc_rHand;
//        tasks.push_back(Task("left foot tracking task", 50, RHandTracking_A, RHandTracking_b));





    // left foot tracking task
    MatrixNd LFootTracking_A = MatrixNd::Zero(6, N_ + 6 * Nc_);
    VectorNd LFootTracking_b = VectorNd::Zero(6);
    LFootTracking_A << world_J_lSole_, MatrixNd::Zero(6, 6 * Nc_);
    LFootTracking_b = -world_Acc_lSole_;
    tasks.push_back(Task("left foot tracking task", 100, LFootTracking_A, LFootTracking_b));

    // right foot tracking tack
    MatrixNd RFootTracking_A = MatrixNd::Zero(6, N_ + 6 * Nc_);
    VectorNd RFootTracking_b = VectorNd::Zero(6);
    RFootTracking_A << world_J_rSole_, MatrixNd::Zero(6, 6 * Nc_);
    RFootTracking_b = -world_Acc_rSole_;
    tasks.push_back(Task("right foot tracking task", 100, RFootTracking_A, RFootTracking_b));




    // combine all tasks
    MatrixNd A = MatrixNd::Zero(0, N_ + 6 * Nc_);
    VectorNd b = VectorNd::Zero(0);
    for (auto task:tasks) {
        A = vstack(A, task.weight * task.A);
        b = concaternate(b, task.weight * task.b);
    }




    //  ############################
    //  #  Inequality constraints  #
    //  ############################
    Matrix3d w_R_r = RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_, getBodyId("RFoot")).transpose();
    Matrix3d w_R_l = RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_, getBodyId("LFoot")).transpose();
    Matrix3d r_R_w = w_R_r.transpose();
    Matrix3d l_R_w = w_R_l.transpose();
    MatrixNd r_S_w = MatrixNd::Zero(6, 6);
    MatrixNd l_S_w = MatrixNd::Zero(6, 6);
    r_S_w << r_R_w, Matrix3d::Zero(), Matrix3d::Zero(), r_R_w;
    l_S_w << l_R_w, Matrix3d::Zero(), Matrix3d::Zero(), l_R_w;
    MatrixNd S_lambda = MatrixNd::Zero(12, 12);
    S_lambda << r_S_w, MatrixNd::Zero(6, 6), MatrixNd::Zero(6, 6), l_S_w;

    MatrixNd zmpConsX = MatrixNd::Zero(8, N_ + 6 * Nc_);
    zmpConsX << MatrixNd::Zero(8, N_), zmpConsLambda_ * S_lambda;

    MatrixNd frictionConsX = MatrixNd::Zero(8, N_ + 6 * Nc_);
    frictionConsX << MatrixNd::Zero(8, N_), frictionConsLambda_ * S_lambda;

    MatrixNd unilateralConsX = MatrixNd::Zero(2, N_ + 6 * Nc_);
    unilateralConsX << MatrixNd::Zero(2, N_), unilateralConsLambda_ * S_lambda;

    MatrixNd inequalityConsMatrix = MatrixNd::Zero(18, N_ + 6 * Nc_);
    inequalityConsMatrix << zmpConsX, frictionConsX, unilateralConsX;
    VectorNd inequalityConsVector = VectorNd::Zero(18);


    //  ############################
    //  #   Equality constraints   #
    //  ############################
    // dynamic constraints
    MatrixNd dynamicConsMatrix = MatrixNd::Zero(6, N_ + 6 * Nc_);
    VectorNd dynamicConsVector = VectorNd::Zero(6);
    dynamicConsMatrix << inertiaMatrix_.block(0, 0, 6, N_), -contactJacobian_.block(0, 0, 6 * Nc_, 6).transpose();
    dynamicConsVector << -nonlinearEffects_.head(6);


    MatrixNd equalityConsMatrix = MatrixNd::Zero(6, N_ + 6 * Nc_);
    VectorNd equalityConsVector = VectorNd::Zero(6);
    equalityConsMatrix << dynamicConsMatrix;
    equalityConsVector << dynamicConsVector;


    //  ############################
    //  #   Solve the QP problem   #
    //  ############################
    MatrixNd H = MatrixNd::Zero(N_ + 6 * Nc_, N_ + 6 * Nc_);
    VectorNd f = VectorNd::Zero(N_ + 6 * Nc_);
    MatrixNd G_ineq = MatrixNd::Zero(9 * Nc_, N_ + 6 * Nc_);
    VectorNd h_ineq = VectorNd::Zero(9 * Nc_);
    MatrixNd G_eq = MatrixNd::Zero(6 + 6 * Nc_, N_ + 6 * Nc_);
    VectorNd h_eq = VectorNd::Zero(6 + 6 * Nc_);


    H = A.transpose() * A;
    f = -A.transpose() * b;
    G_ineq = inequalityConsMatrix;
    h_ineq = inequalityConsVector;
    G_eq = equalityConsMatrix;
    h_eq = equalityConsVector;

//        std::cout << "H:   " << H << std::endl;
//        std::cout << "f:   " << f.transpose() << std::endl;
//        std::cout << "G_ineq:   " << G_ineq << std::endl;
//        std::cout << "h_ineq:   " << h_ineq.transpose() << std::endl;
//        std::cout << "G_eq:   " << G_eq << std::endl;
//        std::cout << "h_eq:   " << h_eq.transpose() << std::endl;

    double solution = 0;
    VectorNd x = VectorNd::Zero(N_ + 6 * Nc_);
    solution = quadprog(H, f, G_ineq, h_ineq, G_eq, h_eq, x);
    VectorNd qdd = x.head(N_);
    VectorNd lambda = x.tail(6 * Nc_);

//        std::cout << "x:   " << x.transpose() << std::endl;
//        std::cout << "qdd:   " << endl << qdd.transpose() << std::endl;
//        std::cout << "lambda:   " << endl << lambda.transpose() << std::endl;

//        cout << "pelvis_orientation_A*x: " << (pelvis_orientation_A*x).transpose() << endl;

    //  ############################
    //  #      inverse dynamics    #
    //  ############################
    VectorNd tau_N = VectorNd::Zero(N_);
    VectorNd tau_n = VectorNd::Zero(n_);
    tau_N = inertiaMatrix_ * qdd + nonlinearEffects_ - contactJacobian_.transpose() * lambda;
    tau_n = tau_N.tail(n_);

//        std::cout << "tau_N:   " << endl << tau_N.transpose() << std::endl;
//        std::cout << "tau_n:   " << endl << tau_n.transpose() << std::endl;

    return tau_n;
}



int InverseDynamicController::getBodyId(const std::string name) {
    std::vector<std::string>::iterator it = std::find(bodies_.begin(), bodies_.end(), name);
    if (it == bodies_.end()) {
        std::cerr << "Error: Body Id not found! Please check body name!" << std::endl;
    } else {
        return it - bodies_.begin();
    }

}

int InverseDynamicController::getJointId(const std::string name) {
    std::vector<std::string>::iterator it = std::find(joints_.begin(), joints_.end(), name);
    if (it == joints_.end()) {
        std::cerr << "Error: Joint Id not found! Please check joint name!" << std::endl;
    } else {
        return it - joints_.begin();
    }

}


RigidBodyDynamics::Math::MatrixNd InverseDynamicController::jointsSelectMatrix(int headIndex, int tailIndex) {
    int NumJoints = tailIndex - headIndex + 1;
    RigidBodyDynamics::Math::MatrixNd S = RigidBodyDynamics::Math::MatrixNd::Zero(NumJoints, N_ + 6 * Nc_);
    S.block(0, headIndex, NumJoints, NumJoints) = RigidBodyDynamics::Math::MatrixNd::Identity(NumJoints, NumJoints);
    return S;
}

RigidBodyDynamics::Math::MatrixNd InverseDynamicController::jointsSelectMatrix(std::string headJointName, std::string tailJointName) {
    int headIndex = getJointId(headJointName);
    int tailIndex = getJointId(tailJointName);
    return jointsSelectMatrix(headIndex, tailIndex);
}