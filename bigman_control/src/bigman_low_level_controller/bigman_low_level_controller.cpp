#include "bigman_low_level_controller/bigman_low_level_controller.h"


LowLevelController::LowLevelController(ros::NodeHandle &nh) : nh(nh), driver(nh) {

    using namespace RigidBodyDynamics;
    using namespace RigidBodyDynamics::Math;

    N = 0;
    n = 0;
    Nc = 0;

    inertiaMatrix = RigidBodyDynamics::Math::MatrixNd::Zero(driver.N, driver.N);
    nonlinearEffects = RigidBodyDynamics::Math::VectorNd::Zero(driver.N);

    world_Acc_lSole = RigidBodyDynamics::Math::SpatialVector::Zero();
    world_Acc_rSole = RigidBodyDynamics::Math::SpatialVector::Zero();
    world_Acc_lHand = RigidBodyDynamics::Math::SpatialVector::Zero();
    world_Acc_rHand = RigidBodyDynamics::Math::SpatialVector::Zero();

    world_J_lSole = RigidBodyDynamics::Math::MatrixNd::Zero(6, driver.N);
    world_J_rSole = RigidBodyDynamics::Math::MatrixNd::Zero(6, driver.N);
    world_J_lHand = RigidBodyDynamics::Math::MatrixNd::Zero(6, driver.N);
    world_J_rHand = RigidBodyDynamics::Math::MatrixNd::Zero(6, driver.N);

    world_Position_lSole = RigidBodyDynamics::Math::Vector3d::Zero();
    world_Position_rSole = RigidBodyDynamics::Math::Vector3d::Zero();
    world_Position_lHand = RigidBodyDynamics::Math::Vector3d::Zero();
    world_Position_rHand = RigidBodyDynamics::Math::Vector3d::Zero();

    world_Rot_lSole = RigidBodyDynamics::Math::Matrix3d::Zero();
    world_Rot_rSole = RigidBodyDynamics::Math::Matrix3d::Zero();
    world_Rot_lHand = RigidBodyDynamics::Math::Matrix3d::Zero();
    world_Rot_rHand = RigidBodyDynamics::Math::Matrix3d::Zero();

    world_Velocity_lSole = RigidBodyDynamics::Math::SpatialVector::Zero();
    world_Velocity_rSole = RigidBodyDynamics::Math::SpatialVector::Zero();
    world_Velocity_lHand = RigidBodyDynamics::Math::SpatialVector::Zero();
    world_Velocity_rHand = RigidBodyDynamics::Math::SpatialVector::Zero();

    contactJacobian = RigidBodyDynamics::Math::MatrixNd::Zero(12, driver.N); // default: double support

    // inequality constraints
    // zmp constraints
    double dx_min = -0.15;
    double dx_max = 0.15;
    double dy_min = -0.07;
    double dy_max = 0.07;
    MatrixNd zmpConsSpatialForce = MatrixNd::Zero(4, 6);
    zmpConsSpatialForce << -1, 0, 0, 0, 0, dy_min,
            1, 0, 0, 0, 0, -dy_max,
            0, 1, 0, 0, 0, dx_min,
            0, -1, 0, 0, 0, -dx_max;

    zmpConsLambda = MatrixNd::Zero(8, 12);
    zmpConsLambda << zmpConsSpatialForce, MatrixNd::Zero(4, 6),
            MatrixNd::Zero(4, 6), zmpConsSpatialForce;


    // friction constraints
    // friction constraints matrix for spatial force [M^T,F^T]^T = [Mx,My,Mz,Fx,Fy,Fz]^T
    double mu = 0.7;
    MatrixNd frictionConsSpatialForce = MatrixNd::Zero(4, 6);
    frictionConsSpatialForce << 0, 0, 0, 1, 0, -mu,
            0, 0, 0, -1, 0, -mu,
            0, 0, 0, 0, 1, -mu,
            0, 0, 0, 0, -1, -mu;
    frictionConsLambda = MatrixNd::Zero(8, 12);
    frictionConsLambda << frictionConsSpatialForce, MatrixNd::Zero(4, 6),
            MatrixNd::Zero(4, 6), frictionConsSpatialForce;

    // unilateral constraints
    MatrixNd unilateralConsSpatialForce = MatrixNd::Zero(1, 6);
    unilateralConsSpatialForce << 0, 0, 0, 0, 0, -1;
    unilateralConsLambda = MatrixNd::Zero(2, 12);
    unilateralConsLambda << unilateralConsSpatialForce, MatrixNd::Zero(1, 6),
            MatrixNd::Zero(1, 6), unilateralConsSpatialForce;


}

void LowLevelController::updateControllerStates() {
    driver.updateRobotStates();

    inertiaMatrix.setZero();
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(driver.model, driver.q, inertiaMatrix);

    nonlinearEffects.setZero();
    NonlinearEffects(driver.model, driver.q, driver.qd, nonlinearEffects);

    world_Acc_lSole = RigidBodyDynamics::CalcPointAcceleration6D(driver.model,
                                                    driver.q,
                                                    driver.qd,
                                                    RigidBodyDynamics::Math::VectorNd::Zero(driver.N),
                                                    driver.getBodyId("LFoot"),
                                                    RigidBodyDynamics::Math::Vector3d(0, 0, -0.1435));
    world_Acc_rSole = RigidBodyDynamics::CalcPointAcceleration6D(driver.model,
                                                    driver.q,
                                                    driver.qd,
                                                    RigidBodyDynamics::Math::VectorNd::Zero(driver.N),
                                                    driver.getBodyId("RFoot"),
                                                    RigidBodyDynamics::Math::Vector3d(0, 0, -0.1435));

    world_Acc_lHand = RigidBodyDynamics::CalcPointAcceleration6D(driver.model,
                                                    driver.q,
                                                    driver.qd,
                                                    RigidBodyDynamics::Math::VectorNd::Zero(driver.N),
                                                    driver.getBodyId("LWrMot3"),
                                                    RigidBodyDynamics::Math::Vector3d::Zero());
    world_Acc_rHand = RigidBodyDynamics::CalcPointAcceleration6D(driver.model,
                                                    driver.q,
                                                    driver.qd,
                                                    RigidBodyDynamics::Math::VectorNd::Zero(driver.N),
                                                    driver.getBodyId("RWrMot3"),
                                                    RigidBodyDynamics::Math::Vector3d::Zero());


    world_J_lSole.setZero();
    world_J_rSole.setZero();
    world_J_lHand.setZero();
    world_J_rHand.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(driver.model,
                              driver.q,
                              driver.getBodyId("LFoot"),
                              RigidBodyDynamics::Math::Vector3d(0, 0, -0.1435),
                              world_J_lSole);
    RigidBodyDynamics::CalcPointJacobian6D(driver.model,
                              driver.q,
                              driver.getBodyId("RFoot"),
                              RigidBodyDynamics::Math::Vector3d(0, 0, -0.1435),
                              world_J_rSole);
    RigidBodyDynamics::CalcPointJacobian6D(driver.model,
                              driver.q,
                              driver.getBodyId("LWrMot3"),
                              RigidBodyDynamics::Math::Vector3d::Zero(),
                              world_J_lHand);
    RigidBodyDynamics::CalcPointJacobian6D(driver.model,
                              driver.q,
                              driver.getBodyId("RWrMot3"),
                              RigidBodyDynamics::Math::Vector3d::Zero(),
                              world_J_rHand);


    world_Position_lSole = RigidBodyDynamics::CalcBodyToBaseCoordinates(driver.model,
                                                           driver.q,
                                                           driver.getBodyId("LFoot"),
                                                           RigidBodyDynamics::Math::Vector3d(0, 0, -0.1435));
    world_Position_rSole = RigidBodyDynamics::CalcBodyToBaseCoordinates(driver.model,
                                                           driver.q,
                                                           driver.getBodyId("RFoot"),
                                                           RigidBodyDynamics::Math::Vector3d(0, 0, -0.1435));
    world_Position_lHand = RigidBodyDynamics::CalcBodyToBaseCoordinates(driver.model,
                                                           driver.q,
                                                           driver.getBodyId("LWrMot3"),
                                                           RigidBodyDynamics::Math::Vector3d::Zero());
    world_Position_rHand = RigidBodyDynamics::CalcBodyToBaseCoordinates(driver.model,
                                                           driver.q,
                                                           driver.getBodyId("RWrMot3"),
                                                           RigidBodyDynamics::Math::Vector3d::Zero());


    world_Rot_lSole = RigidBodyDynamics::CalcBodyWorldOrientation(driver.model, driver.q, driver.getBodyId("LFoot")).transpose();
    world_Rot_rSole = RigidBodyDynamics::CalcBodyWorldOrientation(driver.model, driver.q, driver.getBodyId("RFoot")).transpose();
    world_Rot_lHand = RigidBodyDynamics::CalcBodyWorldOrientation(driver.model, driver.q,
                                                     driver.getBodyId("LWrMot3")).transpose();
    world_Rot_rHand = RigidBodyDynamics::CalcBodyWorldOrientation(driver.model, driver.q,
                                                     driver.getBodyId("RWrMot3")).transpose();


    world_Velocity_lSole = RigidBodyDynamics::CalcPointVelocity6D(driver.model,
                                                     driver.q,
                                                     driver.qd,
                                                     driver.getBodyId("LFoot"),
                                                     RigidBodyDynamics::Math::Vector3d(0, 0, -0.1435));
    world_Velocity_rSole = RigidBodyDynamics::CalcPointVelocity6D(driver.model,
                                                     driver.q,
                                                     driver.qd,
                                                     driver.getBodyId("RFoot"),
                                                     RigidBodyDynamics::Math::Vector3d(0, 0, -0.1435));
    world_Velocity_lHand = RigidBodyDynamics::CalcPointVelocity6D(driver.model,
                                                     driver.q,
                                                     driver.qd,
                                                     driver.getBodyId("LWrMot3"),
                                                     RigidBodyDynamics::Math::Vector3d::Zero());
    world_Velocity_rHand = RigidBodyDynamics::CalcPointVelocity6D(driver.model,
                                                     driver.q,
                                                     driver.qd,
                                                     driver.getBodyId("RWrMot3"),
                                                     RigidBodyDynamics::Math::Vector3d::Zero());


}


RigidBodyDynamics::Math::VectorNd LowLevelController::inverseDynamic(HighLevelCommand highLevelCommand) {
    using namespace std;
    using namespace RigidBodyDynamics;
    using namespace RigidBodyDynamics::Math;

    N = driver.N; // number of all joints
    n = driver.n; // number of actuated joints


    // overwrite with commanded contact state
    switch (highLevelCommand.contactState) {
        case noSupport: {
            Nc = 0;
            break;
        }
        case leftSupport: {
            Nc = 1;
            contactJacobian.resize(6, N);
            contactJacobian << world_J_lSole;
            break;
        }
        case rightSupport: {
            Nc = 1;
            contactJacobian.resize(6, N);
            contactJacobian << world_J_rSole;
            break;
        }
        case doubleSupport: {
            Nc = 2;
            contactJacobian.resize(12, N);
            contactJacobian << world_J_rSole, world_J_lSole;
            break;
        }
    }



    // container for all tasks
    std::vector<Task> tasks = {};

    // least square task
    MatrixNd ls_A = MatrixNd::Identity(N + 6 * Nc, N + 6 * Nc);
    VectorNd ls_b = VectorNd::Zero(N + 6 * Nc);
    tasks.push_back(Task("least square task", 1, ls_A, ls_b));

    // CoM->GRF task
    Vector3d desired_com_acc = Vector3d::Zero();
    desired_com_acc = 10 * (highLevelCommand.CoMPosRef - driver.com) - 1 * (driver.com_velocity);
    Vector3d desired_Ld = Vector3d::Zero();
    desired_Ld = -3 * (driver.angular_momentum);

    Vector3d pr = Vector3d::Zero();
    Vector3d pl = Vector3d::Zero();
    pr = world_Position_rSole - driver.com;
    pl = world_Position_lSole - driver.com;

    Matrix3d Prx = Matrix3d::Zero();
    Matrix3d Plx = Matrix3d::Zero();
    Prx = skew(pr);
    Plx = skew(pl);

    MatrixNd Tr = MatrixNd::Zero(6, 6);
    MatrixNd Tl = MatrixNd::Zero(6, 6);
    Tr << Matrix3d::Zero(), Matrix3d::Identity(), Matrix3d::Identity(), Prx;
    Tl << Matrix3d::Zero(), Matrix3d::Identity(), Matrix3d::Identity(), Plx;

    MatrixNd com_A = MatrixNd::Zero(6, N + 6 * Nc);
    VectorNd com_b = VectorNd::Zero(6);

    com_A << MatrixNd::Zero(6, N), Tr, Tl;
    com_b << driver.mass * (desired_com_acc - driver.G), desired_Ld;

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
            double dr = (world_Position_rSole.head(2) - driver.com.head(2)).norm();
            double dl = (world_Position_lSole.head(2) - driver.com.head(2)).norm();
            double wr = dl / (dr + dl);
            double wl = dr / (dr + dl);
            MatrixNd GRF_dis_A = MatrixNd::Zero(6, N + 6 * Nc);
            VectorNd GRF_dis_b = VectorNd::Zero(6);
            GRF_dis_A << MatrixNd::Zero(6, N), wl * MatrixNd::Identity(6, 6), -wr * MatrixNd::Identity(6, 6);
            tasks.push_back(Task("GRF distribution task", 100, GRF_dis_A, GRF_dis_b));
            break;
        }


    }



    // pelvis position

    // pelvis orientation
    MatrixNd pelvis_orientation_A = MatrixNd::Zero(3, N + 6 * Nc);
    VectorNd pelvis_orientation_b = VectorNd::Zero(3);
    pelvis_orientation_A << jointsSelectMatrix("rx", "rz");
    VectorNd pelvis_orientation_des = VectorNd::Zero(3);
    pelvis_orientation_b << 500 * (pelvis_orientation_des - driver.q.segment(3, 3)) - 10 * driver.qd.segment(3, 3);

    tasks.push_back(Task("pelvis orientation task", 100, pelvis_orientation_A, pelvis_orientation_b));

//        cout << "pelvis_orientation_b: " << pelvis_orientation_b.transpose() << endl;


//        // pelvis task
//        MatrixNd pelvis_A = MatrixNd::Zero(6, N + 6 * Nc);
//        VectorNd pelvis_b = VectorNd::Zero(6);
//        pelvis_A << MatrixNd::Identity(6, 6), MatrixNd::Zero(6, N + 6 * Nc - 6);
//        VectorNd pelvis_des = VectorNd::Zero(6);
//        pelvis_des << driver.q.head(3), 0, 0, 0;
//        pelvis_b << 500 * (pelvis_des - driver.q.head(6)) - 10 * driver.qd.head(6);
//        cout << "pelvis_b: " << pelvis_b.transpose() << endl;
//
//        tasks.push_back(Task("pelvis task", 100, pelvis_A, pelvis_b));


    // upperbody task
    MatrixNd upperbody_A = MatrixNd::Zero(19, N + 6 * Nc);
    VectorNd upperbody_b = VectorNd::Zero(19);
    upperbody_A << MatrixNd::Zero(19, 18), MatrixNd::Identity(19, 19), MatrixNd::Zero(19, 12);
    VectorNd upperbodyConfigDes = VectorNd::Zero(19);
    //upperbodyConfigDes(6) = deg2rad(-45);
    //upperbodyConfigDes(15) = deg2rad(-45);
    upperbody_b << 100 * (upperbodyConfigDes - driver.q.segment(18, 19)) - 10 * driver.qd.segment(18, 19);
    tasks.push_back(Task("upperbody task", 70, upperbody_A, upperbody_b));



//        // head task
//        MatrixNd head_A = jointsSelectMatrix("NeckYawj", "NeckPitchj");
//        VectorNd head_b = VectorNd::Zero(2);
//        VectorNd headQDes = VectorNd::Zero(2);
//        headQDes << deg2rad(30), deg2rad(-0);
//        head_b << jointsPD(driver.q.segment(driver.getJointId("NeckYawj"), 2),
//                           driver.qd.segment(driver.getJointId("NeckPitchj"), 2),
//                           headQDes);
//        tasks.push_back(Task("head task", 30, head_A, head_b));






//        // left hand tracking task
//        MatrixNd LHandTracking_A = MatrixNd::Zero(6, N + 6 * Nc);
//        VectorNd LHandTracking_b = VectorNd::Zero(6);
//        LHandTracking_A << world_J_lHand, MatrixNd::Zero(6, 6 * Nc);
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
//        MatrixNd RHandTracking_A = MatrixNd::Zero(6, N + 6 * Nc);
//        VectorNd RHandTracking_b = VectorNd::Zero(6);
//        RHandTracking_A << world_J_rHand, MatrixNd::Zero(6, 6 * Nc);
//        SpatialVector cmdAccSpatialRHand = cartesianPD(world_Position_rHand, world_Rot_rHand,
//                                                  world_Velocity_rHand.tail(3), world_Velocity_rHand.head(3),
//                                                  Vector3d(0.12, -0.5, 0.8), Matrix3d::Identity());
//
//        RHandTracking_b = cmdAccSpatialRHand - world_Acc_rHand;
//        tasks.push_back(Task("left foot tracking task", 50, RHandTracking_A, RHandTracking_b));





    // left foot tracking task
    MatrixNd LFootTracking_A = MatrixNd::Zero(6, N + 6 * Nc);
    VectorNd LFootTracking_b = VectorNd::Zero(6);
    LFootTracking_A << world_J_lSole, MatrixNd::Zero(6, 6 * Nc);
    LFootTracking_b = -world_Acc_lSole;
    tasks.push_back(Task("left foot tracking task", 100, LFootTracking_A, LFootTracking_b));

    // right foot tracking tack
    MatrixNd RFootTracking_A = MatrixNd::Zero(6, N + 6 * Nc);
    VectorNd RFootTracking_b = VectorNd::Zero(6);
    RFootTracking_A << world_J_rSole, MatrixNd::Zero(6, 6 * Nc);
    RFootTracking_b = -world_Acc_rSole;
    tasks.push_back(Task("right foot tracking task", 100, RFootTracking_A, RFootTracking_b));




    // combine all tasks
    MatrixNd A = MatrixNd::Zero(0, N + 6 * Nc);
    VectorNd b = VectorNd::Zero(0);
    for (auto task:tasks) {
        A = vstack(A, task.weight * task.A);
        b = concaternate(b, task.weight * task.b);
    }




    //  ############################
    //  #  Inequality constraints  #
    //  ############################
    Matrix3d w_R_r = RigidBodyDynamics::CalcBodyWorldOrientation(driver.model, driver.q, driver.getBodyId("RFoot")).transpose();
    Matrix3d w_R_l = RigidBodyDynamics::CalcBodyWorldOrientation(driver.model, driver.q, driver.getBodyId("LFoot")).transpose();
    Matrix3d r_R_w = w_R_r.transpose();
    Matrix3d l_R_w = w_R_l.transpose();
    MatrixNd r_S_w = MatrixNd::Zero(6, 6);
    MatrixNd l_S_w = MatrixNd::Zero(6, 6);
    r_S_w << r_R_w, Matrix3d::Zero(), Matrix3d::Zero(), r_R_w;
    l_S_w << l_R_w, Matrix3d::Zero(), Matrix3d::Zero(), l_R_w;
    MatrixNd S_lambda = MatrixNd::Zero(12, 12);
    S_lambda << r_S_w, MatrixNd::Zero(6, 6), MatrixNd::Zero(6, 6), l_S_w;

    MatrixNd zmpConsX = MatrixNd::Zero(8, N + 6 * Nc);
    zmpConsX << MatrixNd::Zero(8, N), zmpConsLambda * S_lambda;

    MatrixNd frictionConsX = MatrixNd::Zero(8, N + 6 * Nc);
    frictionConsX << MatrixNd::Zero(8, N), frictionConsLambda * S_lambda;

    MatrixNd unilateralConsX = MatrixNd::Zero(2, N + 6 * Nc);
    unilateralConsX << MatrixNd::Zero(2, N), unilateralConsLambda * S_lambda;

    MatrixNd inequalityConsMatrix = MatrixNd::Zero(18, N + 6 * Nc);
    inequalityConsMatrix << zmpConsX, frictionConsX, unilateralConsX;
    VectorNd inequalityConsVector = VectorNd::Zero(18);


    //  ############################
    //  #   Equality constraints   #
    //  ############################
    // dynamic constraints
    MatrixNd dynamicConsMatrix = MatrixNd::Zero(6, N + 6 * Nc);
    VectorNd dynamicConsVector = VectorNd::Zero(6);
    dynamicConsMatrix << inertiaMatrix.block(0, 0, 6, N), -contactJacobian.block(0, 0, 6 * Nc, 6).transpose();
    dynamicConsVector << -nonlinearEffects.head(6);


    MatrixNd equalityConsMatrix = MatrixNd::Zero(6, N + 6 * Nc);
    VectorNd equalityConsVector = VectorNd::Zero(6);
    equalityConsMatrix << dynamicConsMatrix;
    equalityConsVector << dynamicConsVector;


    //  ############################
    //  #   Solve the QP problem   #
    //  ############################
    MatrixNd H = MatrixNd::Zero(N + 6 * Nc, N + 6 * Nc);
    VectorNd f = VectorNd::Zero(N + 6 * Nc);
    MatrixNd G_ineq = MatrixNd::Zero(9 * Nc, N + 6 * Nc);
    VectorNd h_ineq = VectorNd::Zero(9 * Nc);
    MatrixNd G_eq = MatrixNd::Zero(6 + 6 * Nc, N + 6 * Nc);
    VectorNd h_eq = VectorNd::Zero(6 + 6 * Nc);


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
    VectorNd x = VectorNd::Zero(N + 6 * Nc);
    solution = quadprog(H, f, G_ineq, h_ineq, G_eq, h_eq, x);
    VectorNd qdd = x.head(N);
    VectorNd lambda = x.tail(6 * Nc);

//        std::cout << "x:   " << x.transpose() << std::endl;
//        std::cout << "qdd:   " << endl << qdd.transpose() << std::endl;
//        std::cout << "lambda:   " << endl << lambda.transpose() << std::endl;

//        cout << "pelvis_orientation_A*x: " << (pelvis_orientation_A*x).transpose() << endl;

    //  ############################
    //  #      inverse dynamics    #
    //  ############################
    VectorNd tau_N = VectorNd::Zero(N);
    VectorNd tau_n = VectorNd::Zero(n);
    tau_N = inertiaMatrix * qdd + nonlinearEffects - contactJacobian.transpose() * lambda;
    tau_n = tau_N.tail(n);

//        std::cout << "tau_N:   " << endl << tau_N.transpose() << std::endl;
//        std::cout << "tau_n:   " << endl << tau_n.transpose() << std::endl;

    return tau_n;

}

RigidBodyDynamics::Math::MatrixNd LowLevelController::jointsSelectMatrix(int headIndex, int tailIndex) {
    int NumJoints = tailIndex - headIndex + 1;
    RigidBodyDynamics::Math::MatrixNd S = RigidBodyDynamics::Math::MatrixNd::Zero(NumJoints, N + 6 * Nc);
    S.block(0, headIndex, NumJoints, NumJoints) = RigidBodyDynamics::Math::MatrixNd::Identity(NumJoints, NumJoints);
    return S;
}

RigidBodyDynamics::Math::MatrixNd LowLevelController::jointsSelectMatrix(std::string headJointName, std::string tailJointName) {
    int headIndex = driver.getJointId(headJointName);
    int tailIndex = driver.getJointId(tailJointName);
    return jointsSelectMatrix(headIndex, tailIndex);
}

void LowLevelController::homing() {
    ros::Time begin = ros::Time::now();
    while ((ros::Time::now() - begin).toSec() < 2) {
        driver.publishCommand(RigidBodyDynamics::Math::VectorNd::Zero(driver.n),
                              RigidBodyDynamics::Math::VectorNd::Zero(driver.n));
    }


    ros::ServiceClient resetGazeboServiceClient = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    std_srvs::Empty empty;

    ros::Duration(1).sleep();
    resetGazeboServiceClient.call(empty);
    ros::Duration(1).sleep();
    resetGazeboServiceClient.call(empty);
    ros::Duration(1).sleep();
}

void LowLevelController::homingSquat() {
    using namespace std;
    using namespace Eigen;
    using namespace ecl;

    homing();

    Eigen::Vector3d PosLAnkle;
    Eigen::Vector3d PosRAnkle;
    BigmanLegFK(driver.q.segment(6, 6), "L", PosLAnkle);
    BigmanLegFK(driver.q.segment(12, 6), "R", PosRAnkle);

    Eigen::Vector3d ankleShift(0.0, 0.0, 0.1);
    Eigen::Vector3d PosLAnkleDes = PosLAnkle + ankleShift;
    Eigen::Vector3d PosRAnkleDes = PosRAnkle + ankleShift;

    double t0 = 0;
    double te = 2;

    CubicSpline3D LLegCubic(t0, te, PosLAnkle, PosLAnkleDes);
    CubicSpline3D RLegCubic(t0, te, PosRAnkle, PosRAnkleDes);

    ros::Time begin = ros::Time::now();
    while ((ros::Time::now() - begin).toSec() < 2) {
        double tCur = (ros::Time::now() - begin).toSec();

        Vector3d PosLAnkleCur = LLegCubic(tCur);
        Vector3d PosRAnkleCur = RLegCubic(tCur);

        VectorXd qLLeg = VectorXd::Zero(6);
        VectorXd qRLeg = VectorXd::Zero(6);
        BigmanLegIK(PosLAnkleCur, Matrix3d::Identity(), "L", qLLeg);
        BigmanLegIK(PosRAnkleCur, Matrix3d::Identity(), "R", qRLeg);

        VectorXd qCmd = VectorXd::Zero(driver.n);
        qCmd.segment(0, 6) = qLLeg;
        qCmd.segment(6, 6) = qRLeg;

        driver.publishCommand(qCmd, VectorXd::Zero(driver.n));
    }



}