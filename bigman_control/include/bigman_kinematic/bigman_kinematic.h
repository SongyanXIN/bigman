#ifndef BIGMAN_KINEMATIC_H
#define BIGMAN_KINEMATIC_H

#include <math.h>
#include <string>
#include <iostream>
#include <Eigen/Dense>

Eigen::Matrix3d rotx(double angle);
Eigen::Matrix3d roty(double angle);
Eigen::Matrix3d rotz(double angle);
bool BigmanLegFK( Eigen::VectorXd q, std::string leg, Eigen::Vector3d &anklePos);
bool BigmanLegIK( Eigen::Vector3d anklePos, Eigen::Matrix3d ankleRot, std::string leg, Eigen::VectorXd &q);

#endif // BIGMAN_KINEMATIC_H
