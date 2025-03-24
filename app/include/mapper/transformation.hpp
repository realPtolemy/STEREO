#ifndef TRANSFORMATION_HPP
#define TRANSFORMATION_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

class Transformation
{
public:
    Transformation();
    Transformation(Eigen::Matrix4d T);
    ~Transformation();
    void inverse();
private:
    Eigen::Matrix4f T_;
    Eigen::Matrix3f R_;
};

// class Quaternion
// {
// public:
//     Quaternion();
//     ~Quaternion();
// private:
//     Eigen::Quaternionf q_;
// };

#endif