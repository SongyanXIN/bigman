#include "bigman_driver/bigman_driver.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "bigman_driver_node");
    ros::NodeHandle nh;
    int frequency = 500;

    BigmanDriver bigmanDriver(nh);

    ros::Rate rate(frequency);
    while (ros::ok()) {
        bigmanDriver.updateRobotStates();
        bigmanDriver.printRobotStates();
        RigidBodyDynamics::Math::VectorNd q_cmd = RigidBodyDynamics::Math::VectorNd::Zero(bigmanDriver.N);
        RigidBodyDynamics::Math::VectorNd tau_cmd = RigidBodyDynamics::Math::VectorNd::Zero(bigmanDriver.N);
        q_cmd[bigmanDriver.getJointId("LKneeSag")] = 45 * 3.14 / 180;
        bigmanDriver.publishCommand(q_cmd, tau_cmd);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

