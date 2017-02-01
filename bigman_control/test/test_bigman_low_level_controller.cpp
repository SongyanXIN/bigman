#include <ros/package.h>
#include "loadCSV/loadCSV.hpp"
#include "bigman_low_level_controller/bigman_low_level_controller.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "test_bigman_low_level_controller_node");
    ros::NodeHandle nh;

    LowLevelController lowLevelController(nh);
    lowLevelController.homingSquat();


    int hz = 500;
    int loopCount = 0;
    ros::Rate rate(hz);
    while (ros::ok()) {
        lowLevelController.updateControllerStates();

        RigidBodyDynamics::Math::Vector3d CoMPosRef = RigidBodyDynamics::Math::Vector3d::Zero();
        CoMPosRef[2] = 1.0;

        HighLevelCommand highLevelCommand;
        highLevelCommand.CoMPosRef = CoMPosRef;

        RigidBodyDynamics::Math::VectorNd tauCmd = RigidBodyDynamics::Math::VectorNd::Zero(lowLevelController.driver.n);
        tauCmd = lowLevelController.inverseDynamic(highLevelCommand);

        RigidBodyDynamics::Math::VectorNd qCmd = RigidBodyDynamics::Math::VectorNd::Zero(lowLevelController.driver.n);
        qCmd = lowLevelController.driver.q.tail(lowLevelController.driver.n);

        lowLevelController.driver.publishCommand(qCmd, tauCmd);

        ros::spinOnce();
        loopCount = loopCount + 1;

        rate.sleep();

    }

    return 0;
}





















