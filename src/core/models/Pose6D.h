#pragma once

struct Pose {
    double x, y, z;
};

struct Pose6D {
    double x, y, z, rx, ry, rz;
};

struct PickPose {
    double x, y, z, rx, ry, rz;
};
struct JointPose {
    double j1, j2, j3, j4, j5, j6;
};
