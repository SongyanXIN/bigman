#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <iostream>
#include <algorithm>
#include <rbdl/rbdl.h>
#include <angles/angles.h>
#include <ecl/geometry.hpp>

// format for Eigen output
extern Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
extern Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
extern Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
extern Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");


double rad2deg(double rad) {
    double deg;
    deg = rad * 180 / M_PI;
    return deg;
}

double deg2rad(double deg) {
    double rad;
    rad = deg * M_PI / 180;
    return rad;
}

double clip(double n, double lower, double upper) {
    return std::max(lower, std::min(n, upper));
}

double trace(RigidBodyDynamics::Math::Matrix3d Rot) {
    return Rot(0, 0) + Rot(1, 1) + Rot(2, 2);
}

RigidBodyDynamics::Math::Matrix3d skew(RigidBodyDynamics::Math::Vector3d v) {
    RigidBodyDynamics::Math::Matrix3d v_hat;
    v_hat << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return v_hat;
}

RigidBodyDynamics::Math::Vector3d vex(RigidBodyDynamics::Math::Matrix3d v_hat) {
    RigidBodyDynamics::Math::Vector3d v;
    v << 0.5 * (v_hat(2, 1) - v_hat(1, 2)), 0.5 * (v_hat(0, 2) - v_hat(2, 0)), 0.5 * (v_hat(1, 0) - v_hat(0, 1));
    return v;
}

RigidBodyDynamics::Math::VectorNd jointsPD(RigidBodyDynamics::Math::VectorNd curQ,
                                           RigidBodyDynamics::Math::VectorNd curQd,
                                           RigidBodyDynamics::Math::VectorNd refQ,
                                           int kp = 200,
                                           int kd = 1) {
    RigidBodyDynamics::Math::VectorNd cmdQdd;
    cmdQdd = kp * (refQ - curQ) - kd * curQd;
    return cmdQdd;
}

RigidBodyDynamics::Math::Vector3d positionPD(RigidBodyDynamics::Math::Vector3d curPos,
                                             RigidBodyDynamics::Math::Vector3d curVel,
                                             RigidBodyDynamics::Math::Vector3d refPos,
                                             RigidBodyDynamics::Math::Vector3d refVel = RigidBodyDynamics::Math::Vector3d::Zero(),
                                             RigidBodyDynamics::Math::Vector3d refAcc = RigidBodyDynamics::Math::Vector3d::Zero(),
                                             int kp = 200,
                                             int kd = 1) {
    RigidBodyDynamics::Math::Vector3d cmdAcc;
    cmdAcc = kp * (refPos - curPos) + kd * (refVel - curVel) + refAcc;
    return cmdAcc;
}

RigidBodyDynamics::Math::Vector3d rotationPD(RigidBodyDynamics::Math::Matrix3d curRot,
                                             RigidBodyDynamics::Math::Vector3d curOmega,
                                             RigidBodyDynamics::Math::Matrix3d refRot,
                                             RigidBodyDynamics::Math::Vector3d refOmega = RigidBodyDynamics::Math::Vector3d::Zero(),
                                             RigidBodyDynamics::Math::Vector3d refOmegaDot = RigidBodyDynamics::Math::Vector3d::Zero(),
                                             int kp = 200,
                                             int kd = 1) {
    RigidBodyDynamics::Math::Matrix3d cur_R_ref = curRot.transpose() * refRot;
    double angle = acos(clip((trace(cur_R_ref) - 1.0) / 2.0, -1.0, 1.0));
    RigidBodyDynamics::Math::Matrix3d log_SO3_R = angle / (2.0 * sin(angle)) * (cur_R_ref - cur_R_ref.transpose());
    RigidBodyDynamics::Math::Vector3d cmdAccAngular = kp * vex(log_SO3_R) + kd * (refOmega - curOmega) + refOmegaDot;
    return cmdAccAngular;
}

RigidBodyDynamics::Math::SpatialVector cartesianPD(RigidBodyDynamics::Math::Vector3d curPos,
                                                   RigidBodyDynamics::Math::Matrix3d curRot,
                                                   RigidBodyDynamics::Math::Vector3d curVelLinear,
                                                   RigidBodyDynamics::Math::Vector3d curVelAngular,
                                                   RigidBodyDynamics::Math::Vector3d refPos,
                                                   RigidBodyDynamics::Math::Matrix3d refRot,
                                                   RigidBodyDynamics::Math::Vector3d refVelLinear = RigidBodyDynamics::Math::Vector3d::Zero(),
                                                   RigidBodyDynamics::Math::Vector3d refVelAngular = RigidBodyDynamics::Math::Vector3d::Zero(),
                                                   RigidBodyDynamics::Math::Vector3d refAccLinear = RigidBodyDynamics::Math::Vector3d::Zero(),
                                                   RigidBodyDynamics::Math::Vector3d refAccAngular = RigidBodyDynamics::Math::Vector3d::Zero(),
                                                   int kpLinear = 200,
                                                   int kdLinear = 1,
                                                   int kpAngular = 200,
                                                   int kdAngular = 1) {
    RigidBodyDynamics::Math::Vector3d cmdAccLinear = positionPD(curPos, curVelLinear, refPos,
                                                                refVelLinear, refAccLinear,
                                                                kpLinear, kdLinear);
    RigidBodyDynamics::Math::Vector3d cmdAccAngular = rotationPD(curRot, curVelAngular, refRot,
                                                                 refVelAngular, refAccAngular,
                                                                 kpAngular, kdAngular);
    RigidBodyDynamics::Math::SpatialVector cmdAccSpatial;
    cmdAccSpatial << cmdAccAngular, cmdAccLinear;
    return cmdAccSpatial;
}

RigidBodyDynamics::Math::SpatialVector SE3PD(RigidBodyDynamics::Math::Vector3d curPos,
                                             RigidBodyDynamics::Math::Matrix3d curRot,
                                             RigidBodyDynamics::Math::SpatialVector curSpaticalVel,
                                             RigidBodyDynamics::Math::Vector3d refPos,
                                             RigidBodyDynamics::Math::Matrix3d refRot,
                                             RigidBodyDynamics::Math::SpatialVector refSpaticalVel = RigidBodyDynamics::Math::SpatialVector::Zero(),
                                             RigidBodyDynamics::Math::SpatialVector refSpaticalAcc = RigidBodyDynamics::Math::SpatialVector::Zero(),
                                             int kpLinear = 200,
                                             int kdLinear = 1,
                                             int kpAngular = 200,
                                             int kdAngular = 1) {
    RigidBodyDynamics::Math::Vector3d cmdAccLinear = positionPD(curPos, curSpaticalVel.tail(3), refPos,
                                                                refSpaticalVel.tail(3), refSpaticalAcc.tail(3),
                                                                kpLinear, kdLinear);
    RigidBodyDynamics::Math::Vector3d cmdAccAngular = rotationPD(curRot, curSpaticalVel.head(3), refRot,
                                                                 refSpaticalVel.tail(3), refSpaticalAcc.head(3),
                                                                 kpAngular, kdAngular);
    RigidBodyDynamics::Math::SpatialVector cmdAccSpatial;
    cmdAccSpatial << cmdAccAngular, cmdAccLinear;
    return cmdAccSpatial;
}


RigidBodyDynamics::Math::MatrixNd hstack(RigidBodyDynamics::Math::MatrixNd A, RigidBodyDynamics::Math::MatrixNd B) {
    if (A.rows() != B.rows()) {
        std::cerr << "Input Matries' rows do not match! ";
        std::cerr << "A.rows(): " << A.rows() << "!= B.rows(): " << B.rows();
    }
    RigidBodyDynamics::Math::MatrixNd C(A.rows(), A.cols() + B.cols());
    C << A, B;
    return C;
}

RigidBodyDynamics::Math::MatrixNd vstack(RigidBodyDynamics::Math::MatrixNd A, RigidBodyDynamics::Math::MatrixNd B) {
    if (A.cols() != B.cols()) {
        std::cerr << "Input Matries' cols do not match! ";
        std::cerr << "A.cols(): " << A.cols() << "!= B.cols(): " << B.cols();
    }
    RigidBodyDynamics::Math::MatrixNd C(A.rows() + B.rows(), A.cols());
    C << A, B;
    return C;
}

RigidBodyDynamics::Math::VectorNd
concaternate(RigidBodyDynamics::Math::VectorNd v1, RigidBodyDynamics::Math::VectorNd v2) {
    RigidBodyDynamics::Math::VectorNd v(v1.size() + v2.size());
    v << v1, v2;
    return v;
}



namespace ecl {
    class CubicSpline3D {
    private:
        CubicSpline xCubic;
        CubicSpline yCubic;
        CubicSpline zCubic;

    public:
        CubicSpline3D(double t0, double te, Eigen::Vector3d P0, Eigen::Vector3d Pe) {
            ecl::Array<double> t_set(2);
            ecl::Array<double> x_set(2);
            ecl::Array<double> y_set(2);
            ecl::Array<double> z_set(2);
            t_set << t0, te;
            x_set << P0[0], Pe[0];
            y_set << P0[1], Pe[1];
            z_set << P0[2], Pe[2];
            xCubic = CubicSpline::Natural(t_set, x_set);
            yCubic = CubicSpline::Natural(t_set, y_set);
            zCubic = CubicSpline::Natural(t_set, z_set);
        }


        Eigen::Vector3d operator()(double t) {
            double x = xCubic(t);
            double y = yCubic(t);
            double z = zCubic(t);
            return Eigen::Vector3d(x, y, z);
        }
    };
}


#endif //PROJECT_UTILS_H
