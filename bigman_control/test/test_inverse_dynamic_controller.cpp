#include "bigman_driver/bigman_driver.h"
#include "inverse_dynamic_controller/inverse_dynamic_controller.h"


void homing(ros::NodeHandle &nh) {
    BigmanDriver driver(nh);
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

void homingSquat(ros::NodeHandle &nh) {
    using namespace std;
    using namespace Eigen;
    using namespace ecl;

    homing(nh);

    BigmanDriver driver(nh);

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

int main(int argc, char **argv) {

    ros::init(argc, argv, "test_inverse_dynamic_controller_node");
    ros::NodeHandle nh;

    BigmanDriver bigman_driver(nh);
    InverseDynamicController inverse_dynamic_controller(nh);

    homingSquat(nh);

    int hz = 500;
    int loop_count = 0;
    ros::Rate rate(hz);
    while (ros::ok()) {
        bigman_driver.updateRobotStates();
        bigman_driver.printRobotStates();

        inverse_dynamic_controller.getRobotStates(bigman_driver.q, bigman_driver.qd);

        RigidBodyDynamics::Math::Vector3d com_pos_ref = RigidBodyDynamics::Math::Vector3d::Zero();
        com_pos_ref[2] = 1.0;
        HighLevelCommand high_level_command;
        high_level_command.CoMPosRef = com_pos_ref;

        RigidBodyDynamics::Math::VectorNd tauCmd = RigidBodyDynamics::Math::VectorNd::Zero(bigman_driver.n);
        tauCmd = inverse_dynamic_controller.CalcJointTorque(high_level_command);

        RigidBodyDynamics::Math::VectorNd qCmd = RigidBodyDynamics::Math::VectorNd::Zero(bigman_driver.n);
        qCmd = bigman_driver.q.tail(bigman_driver.n);

        bigman_driver.publishCommand(qCmd, tauCmd);


        ros::spinOnce();
        loop_count = loop_count + 1;
        std::cout << loop_count << std::endl;
        rate.sleep();

    }

    return 0;
}





















