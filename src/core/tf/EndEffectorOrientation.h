#ifndef ENDEFFECTORORIENTATION_H
#define ENDEFFECTORORIENTATION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

class EndEffectorOrientation {
public:
    static Quaterniond EulerZYXToQuaternion(double roll, double pitch, double yaw);
    static Vector3d QuaternionToEulerZYX(const Quaterniond& q);
    static Quaterniond ApplyYawRotationToEndEffector(const Quaterniond& current_q, double additional_yaw);
};

#endif // ENDEFFECTORORIENTATION_H
