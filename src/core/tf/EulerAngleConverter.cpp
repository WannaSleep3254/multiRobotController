#include "EulerAngleConverter.h"

// 오일러 각(ZYX) -> 회전 행렬 변환
Matrix3d EulerAngleConverter::ConvertEulerZYXToRotationMatrix(double roll_radians, double pitch_radians, double yaw_radians) {
    return AngleAxisd(yaw_radians, Vector3d::UnitZ()).toRotationMatrix() *
           AngleAxisd(pitch_radians, Vector3d::UnitY()).toRotationMatrix() *
           AngleAxisd(roll_radians, Vector3d::UnitX()).toRotationMatrix();
}

// 회전 행렬 -> 오일러 각(ZYX) 변환
Vector3d EulerAngleConverter::ConvertRotationMatrixToEulerZYX(const Matrix3d& rotation_matrix) {
    return rotation_matrix.eulerAngles(2, 1, 0); // ZYX 순서로 오일러 각 추출
}

// 기존 툴 좌표계에서 추가 YAW 회전 적용
Matrix3d EulerAngleConverter::ApplyYawRotationToToolFrame(const Matrix3d& tool_rotation_matrix, double additional_yaw_radians) {
    Matrix3d yaw_rotation_matrix = AngleAxisd(additional_yaw_radians, Vector3d::UnitZ()).toRotationMatrix();
    return tool_rotation_matrix * yaw_rotation_matrix; // 기존 회전 행렬에 추가 YAW 회전 적용
}

// Radian to Degree 변환
double EulerAngleConverter::ConvertRadiansToDegrees(double radians) {
    return radians * (180.0 / EulerAngleConverter::PI);
}

// Degree to Radian 변환
double EulerAngleConverter::ConvertDegreesToRadians(double degrees) {
    return degrees * (EulerAngleConverter::PI / 180.0);
}

// 벡터 정규화 (단위 벡터로 변환)
Vector3d EulerAngleConverter::NormalizeVector(const Vector3d& v) {
    return v.normalized();  // Eigen 내장 함수 사용
}


// 각도(라디안) 정규화 (-π ~ π 범위)
double EulerAngleConverter::NormalizeAngleRadians(double radians) {
    constexpr double TWO_PI = 2.0 * PI;
    return radians - TWO_PI * round(radians / TWO_PI);
}
