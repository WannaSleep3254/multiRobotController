#ifndef ROBOTPOSEVALIDATOR_H
#define ROBOTPOSEVALIDATOR_H
// C++ Standard
#include <cmath>
// Qt
#include <QString>
// 3rd party
#include <Eigen/Dense>
// Project
#include "Pose6D.h"

Eigen::Matrix4d poseToMatrix(double x, double y, double z,
                             double rx, double ry, double rz)
{
    // rx,ry,rz : degree
    double ax = rx * M_PI / 180.0;
    double ay = ry * M_PI / 180.0;
    double az = rz * M_PI / 180.0;

    Eigen::AngleAxisd Rx(ax, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd Ry(ay, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rz(az, Eigen::Vector3d::UnitZ());

    // ⚠ 로봇 매뉴얼 기준 회전 순서 맞추기 (보통 ZYX)
    Eigen::Matrix3d R = (Rz * Ry * Rx).toRotationMatrix();

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = Eigen::Vector3d(x,y,z);
    return T;
}

struct BaseSafetyZone
{
    // ① 원점 근처 “완전 0” 방지 (초기값/에러값)
    double posTolAllZeroMm = 0.5;   // mm
    double angTolAllZeroDeg = 0.5;  // deg

    // ② 베이스 주변 금지영역 (원통)  r < R 이고 z < Zmax 이면 금지
    double forbidRadiusMm = 250.0;  // 베이스 반경 (예: 200~400mm)
    double forbidZmaxMm   = 400.0;  // 베이스 근처 높이 (예: 200~600mm)

    // ③ 작업공간 (현장에 맞게)
    double xMin=-1200, xMax=1200;
    double yMin=-1200, yMax=1200;
    double zMin=  50,  zMax=300;
};


static inline bool nearZero(double v, double tol) { return std::abs(v) <= tol; }
static inline bool inRange(double v, double mn, double mx) { return (mn <= v && v <= mx); }


static bool isAllNearZeroPose(const Pose6D& p, double posTolMm, double angTolDeg)
{
    return nearZero(p.x,  posTolMm) && nearZero(p.y,  posTolMm) && nearZero(p.z,  posTolMm) &&
           nearZero(p.rx, angTolDeg) && nearZero(p.ry, angTolDeg) && nearZero(p.rz, angTolDeg);
}

static bool isInWorkspaceXYZ(const Pose6D& p, const BaseSafetyZone& z)
{
    return inRange(p.x, z.xMin, z.xMax) &&
           inRange(p.y, z.yMin, z.yMax) &&
           inRange(p.z, z.zMin, z.zMax);
}

// 베이스 원점(0,0,0) 기준 금지영역: 원통 형태
static bool isInBaseForbiddenCylinder(const Pose6D& p, const BaseSafetyZone& z)
{
    const double r = std::hypot(p.x, p.y);     // sqrt(x^2 + y^2)
    return (r < z.forbidRadiusMm) && (p.z < z.forbidZmaxMm);
}

bool validateTargetPose_Base0(const Pose6D& p, const BaseSafetyZone& z, QString* reasonOut)
{
    auto fail = [&](const QString& r){
        if (reasonOut) *reasonOut = r;
        return false;
    };

    // NaN/Inf 방지
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(p.rx)|| !std::isfinite(p.ry)|| !std::isfinite(p.rz))
        return fail("pose contains NaN/Inf");

    // (0,0,0,0,0,0) 근처 방지
    if (isAllNearZeroPose(p, z.posTolAllZeroMm, z.angTolAllZeroDeg))
        return fail("pose is near all-zero (default/invalid)");

    // 작업공간 범위
    if (!isInWorkspaceXYZ(p, z))
        return fail(QString("pose out of workspace (x=%1,y=%2,z=%3)").arg(p.x).arg(p.y).arg(p.z));

    // 베이스 근처 금지영역
    if (isInBaseForbiddenCylinder(p, z))
        return fail(QString("pose in forbidden base zone (r=%1,z=%2)")
                        .arg(std::hypot(p.x,p.y)).arg(p.z));

    return true;
}


#endif // ROBOTPOSEVALIDATOR_H
