#include "bigman_kinematic/bigman_kinematic.h"

using namespace Eigen;
using namespace std;

const double  PI = 3.14159265359;
//Robot parameters
const Vector3d d0(0,0.06,0), d1(0,0.121032,0), d2(0,0,-0.217872),d3(0,0,-0.356),d4(0,0,-0.4);

Matrix3d rotx(double angle)
{
        //Matrix defined as stattic to accelerate calculation
        Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
        double ct;
        double st;

        ct = cos(angle);
        st = sin(angle);
        result(1,1) = ct;

        result(1,2) = -st;
        result(2,1) = st;
        result(2,2) = ct;

        return result;
}

Matrix3d roty(double angle)
{
        //Matrix defined as stattic to accelerate calculation
        Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
        double ct;
        double st;

        ct = cos(angle);
        st = sin(angle);

        result(0,0) = ct;
        result(0,2) = st;
        result(2,0) = -st;
        result(2,2) = ct;

        return result;
}

Matrix3d rotz(double angle)
{
        //Matrix defined as stattic to accelerate calculation
        Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
        double ct;
        double st;

        ct = cos(angle);
        st = sin(angle);

        result(0,0) = ct;
        result(0,1) = -st;
        result(1,0) = st;
        result(1,1) = ct;

        return result;
}

double twoVectorsAngle( Vector3d first, Vector3d second)
{
  first.normalize();
  second.normalize();

  double s = first.cross(second).norm();
  double c = first.dot(second);
  return atan2(s, c);

}

double twoVectorsAngleSigned( Vector3d first, Vector3d second, Vector3d planeNormalVector)
{
  first.normalize();
  second.normalize();
  planeNormalVector.normalize();

  errno = 0;

  //From some reasons the dot product of normalized vectors was bigger than 1 and that results in nan.
  double dotProduct =  first.dot(second);
  if (dotProduct >  1) dotProduct =  1;
  if (dotProduct < -1) dotProduct = -1;
  double angle = acos( dotProduct );
  if (errno != 0) {
    switch(errno) {
      case EDOM: cout << "Error EDOM " << endl; break;
      case ERANGE: cout << "Error ERANGE " << endl; break;
      case EILSEQ: cout << "Error EILSEQ " << endl; break;
    }
  }

  Vector3d crossP = first.cross(second);

  if (planeNormalVector.dot(crossP) < 0)
      angle = -angle;

  return angle;
}

double triangleAngleFromThreeSides(double closeOne, double closeTwo, double opposite)
{
  double acosValue = (closeOne*closeOne + closeTwo*closeTwo - opposite*opposite) / (2 * closeOne * closeTwo);
  if (acosValue >  1) acosValue =  1;
  if (acosValue < -1) acosValue = -1;
  return acos( acosValue );
}

bool BigmanLegFK( VectorXd q, string leg, Vector3d &anklePos)
{
    Matrix3d R01, R12, R23, R34, R03, R04;
    Vector3d hipPos, kneePos;

    R01 = rotx(q[0]);	// hip roll
    R12 = rotz(q[1]);	// hip yaw
    R23 = roty(q[2]);	// hip pitch
    R34 = roty(q[3]);	// knee
    //R45 = roty(q[4]);	// ankle pitch
    //R56 = rotx(q[5]);	// ankle roll

    if (leg == "L") {
        hipPos = d0+R01*(d1+d2);
    } else if (leg == "R") {
        hipPos = -d0+R01*(-d1+d2);
    }

    R03 = R01*R12*R23;
    R04 = R01*R12*R23*R34;

    kneePos = R03*d3+hipPos;
    anklePos = kneePos+R04*d4; // R45 R56 are not needed to calcuate the ankle position

    anklePos(0) = anklePos(0) + 0.0170;   // 0.0170 - x shift from pelvis

    //Check if there is no nan's inside the vector
    if (anklePos != anklePos) {
        return false;
    }
    return true;
}

bool BigmanLegIK( Vector3d anklePos, Matrix3d ankleRot, string leg, VectorXd &q)
{
  if (q.rows() != 6)
    q.resize(6);//q[6]: hip roll, hip yaw, hip pitch, knee pitch, ankle pitch, ankle roll;

  //Robot parameters
  anklePos(0) = anklePos(0) - 0.0170;   // 0.0170 - x shift from pelvis

  //pelvis ZY plane parameters
  Vector3d XYpelvis_normalVector(1, 0, 0);

  // ==================== HIP ROLL ====================
  //Ankle roll joint x axis before rotation
  Vector3d ankleX_point  = anklePos;
  Vector3d ankleX_vector = ankleRot.block(0,0,3,1); //(1:3,1);

  //Plane and line intersection point
  double ankle_d = (-ankleX_point).dot(XYpelvis_normalVector) / ankleX_vector.dot(XYpelvis_normalVector);
  Vector3d XYpelvis_ankleXIntersection = ankle_d*ankleX_vector + ankleX_point;

  double hipRollToYaw, hipRollToAnkleDist, hr_alpha, hr_beta;

  //Calculate hip roll angle
  if (leg == "L") {
    hipRollToYaw = d1.norm();
    hipRollToAnkleDist = (-d0 + XYpelvis_ankleXIntersection).norm();
    hr_alpha = acos(hipRollToYaw / hipRollToAnkleDist);
    hr_beta  = twoVectorsAngle(-d0, XYpelvis_ankleXIntersection - d0);
    q(0) = (hr_alpha + hr_beta - PI);
  } else if (leg == "R") {
    hipRollToYaw = d1.norm();
    hipRollToAnkleDist = (d0 + XYpelvis_ankleXIntersection).norm();
    hr_alpha = acos(hipRollToYaw / hipRollToAnkleDist);
    hr_beta  = twoVectorsAngle(d0, XYpelvis_ankleXIntersection + d0);
    q(0) = -(hr_alpha + hr_beta - PI);
  }

  // ==================== HIP YAW ====================
  //Normal vector of plane containing ankle X-axis and hip-yaw axis
  Matrix3d hipRollRot = rotx(q(0));
  Matrix3d hipRotation = rotx(q(0));
  Vector3d hipYaw_pos;
  if (leg == "L") {
      hipYaw_pos = d0 + hipRotation * d1;
  } else if (leg == "R") {
      hipYaw_pos = -d0 - hipRotation * d1;
  }

  Vector3d ankleToHipYaw = hipYaw_pos - ankleX_point;
  Vector3d ankleXHipYawPlane_normal = ankleToHipYaw.cross(ankleX_vector);
  //Normal vector conaining XZ-axes of hip yaw plane before yaw rotation
  Vector3d hipYawXZ_normal = hipRollRot.block(0,1,3,1); //(1:3,2);
  //Vector3d hipYaw_Z = hipRollRot.block(0,2,3,1); //(1:3,3);
  q(1) = twoVectorsAngleSigned(hipYawXZ_normal, ankleXHipYawPlane_normal, hipRollRot.block(0,2,3,1)); //(1:3,3));
  hipRotation = hipRotation * rotz(q(1));

  // ==================== KNEE PITCH ====================
  Vector3d hipPitch_pos = hipYaw_pos + hipRotation*d2;
  Vector3d hipPitchToAnkle_vector = ankleX_point - hipPitch_pos;
  if (hipPitchToAnkle_vector.norm() > (d3.norm() + d4.norm() - 0.000)) {
      //cout << leg << ": Knee is getting overstretched. Desired: " << hipPitchToAnkle_vector.norm() << ", Limit" << d3.norm() + d4.norm() - 0.000 << endl;
      //return false;
      q(3) = PI - triangleAngleFromThreeSides(abs(d3(2)), abs(d4(2)), (d3.norm() + d4.norm() - 0.000));
  }
  else
  q(3) = PI - triangleAngleFromThreeSides(abs(d3(2)), abs(d4(2)), hipPitchToAnkle_vector.norm());

  // ==================== HIP PITCH ====================
  Vector3d hipPitchToAnkleXPelvisXYintersection = XYpelvis_ankleXIntersection - hipPitch_pos;
  Vector3d ankleToHipPitch = hipPitch_pos - ankleX_point;
  double hipYaw_ankleX_anklePos_angle = twoVectorsAngleSigned(hipPitchToAnkleXPelvisXYintersection, -ankleToHipPitch, ankleXHipYawPlane_normal);
  q(2) = -triangleAngleFromThreeSides(abs(d3(2)), hipPitchToAnkle_vector.norm(), abs(d4(2)));
  q(2) = q(2) + hipYaw_ankleX_anklePos_angle;

  // ==================== ANKLE PITCH ====================
  hipRotation = hipRotation * roty(q(2));
  Matrix3d kneePitchRot = roty(q(3));
  Matrix3d pelvisToKneePitchRot = hipRotation*kneePitchRot;
  q(4) =  twoVectorsAngleSigned(pelvisToKneePitchRot.block(0,0,3,1), ankleX_vector, pelvisToKneePitchRot.block(0,1,3,1));

  // ==================== ANKLE ROLL ====================
  Matrix3d pelvisToAnklePitchRot = hipRotation*kneePitchRot * roty(q(4));
  q(5) =  twoVectorsAngleSigned(pelvisToAnklePitchRot.block(0,1,3,1), ankleRot.block(0,1,3,1), pelvisToAnklePitchRot.block(0,0,3,1));

  //Check if there is no nan's inside the vector
  if (q != q) {
      return false;
  }
  return true;
}

