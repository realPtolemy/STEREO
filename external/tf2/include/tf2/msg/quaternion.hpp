#ifndef TF2_MSG_QUATERNION_HPP
#define TF2_MSG_QUATERNION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace msg{
    struct Quaternion{
        Eigen::Quaterniond rotation;
    };
}

#endif