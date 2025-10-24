#ifndef EULERANGLECONVERTER_H
#define EULERANGLECONVERTER_H

#include <Eigen/Dense>
#include <qmath.h>

using namespace Eigen;

class EulerAngleConverter {
public:
    // 상수 정의 (M_PI 대신 사용)
    //static constexpr double PI = 3.14159265358979323846;
    static constexpr double PI= M_PI;//atan(1) * 4;

    // 오일러 각(ZYX) -> 회전 행렬 변환
    static Matrix3d ConvertEulerZYXToRotationMatrix(double roll_radians, double pitch_radians, double yaw_radians);

    // 회전 행렬 -> 오일러 각(ZYX) 변환
    static Vector3d ConvertRotationMatrixToEulerZYX(const Matrix3d& rotation_matrix);

    // 기존 툴 좌표계에서 추가 YAW 회전 적용
    static Matrix3d ApplyYawRotationToToolFrame(const Matrix3d& tool_rotation_matrix, double additional_yaw_radians);

    // Radian to Degree 변환
    static double ConvertRadiansToDegrees(double radians);

    // Degree to Radian 변환
    static double ConvertDegreesToRadians(double degrees);

    // 추가된 정규화 함수
    static Vector3d NormalizeVector(const Vector3d& v);
    static double NormalizeAngleRadians(double radians);
};

#endif // EULERANGLECONVERTER_H
