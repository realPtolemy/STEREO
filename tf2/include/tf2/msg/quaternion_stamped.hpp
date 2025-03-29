#ifndef TF2_MSG_QUATERNION_STAMPED_HPP
#define TF2_MSG_QUATERNION_STAMPED_HPP

#include <string>
#include "tf2/time.hpp"
#include "tf2/msg/quaternion.hpp"

namespace msg{
    struct QuaternionStamped{
        // === Header ===
        uint32_t seq = 0;  // Optional: only needed if you care about message order
        tf2::TimePoint timestamp;
        std::string frame_id;

        // === Body ===
        Eigen::Quaterniond quaternion;

        QuaternionStamped()
            : quaternion(Eigen::Quaterniond::Identity()) {}
    };
}

#endif