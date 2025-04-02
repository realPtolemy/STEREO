#ifndef TF2_MSG_TRANSFORM_STAMPED_HPP
#define TF2_MSG_TRANSFORM_STAMPED_HPP

#include <string>
#include "tf2/time.hpp"
#include "tf2/msg/quaternion.hpp"
namespace tf2{
namespace msg{
	struct PoseStamped {
		// === Header ===
		uint32_t seq = 0;  // Optional: only needed if you care about message order
		tf2::TimePoint timestamp;
		std::string frame_id;

		Eigen::Vector3d translation;
		Eigen::Quaterniond rotation;

		PoseStamped()
			: translation(Eigen::Vector3d::Zero()),
			rotation(Eigen::Quaterniond::Identity()) {}
	};
}
}

#endif