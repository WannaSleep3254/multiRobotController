#include "EndEffectorOrientation.h"
#include "qglobal.h"

// 오일러 각(ZYX) -> 쿼터니언 변환
Quaterniond EndEffectorOrientation::EulerZYXToQuaternion(double roll, double pitch, double yaw) {
    return AngleAxisd(yaw, Vector3d::UnitZ()) *
           AngleAxisd(pitch, Vector3d::UnitY()) *
           AngleAxisd(roll, Vector3d::UnitX());
}

// 쿼터니언 -> 오일러 각(ZYX) 변환
Vector3d EndEffectorOrientation::QuaternionToEulerZYX(const Quaterniond& q) {
    return q.toRotationMatrix().eulerAngles(2, 1, 0);
}

// 엔드 이펙터에 추가적인 YAW 회전 적용
Quaterniond EndEffectorOrientation::ApplyYawRotationToEndEffector(const Quaterniond& current_q, double additional_yaw) {
    Q_UNUSED(additional_yaw)

    Quaterniond yaw_q;// = AngleAxisd(additional_yaw, Vector3d::UnitZ());
    return current_q * yaw_q;
}
