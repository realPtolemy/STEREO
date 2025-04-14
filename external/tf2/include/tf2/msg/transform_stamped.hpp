#ifndef TF2_MSG_TRANSFORM_STAMPED_HPP
#define TF2_MSG_TRANSFORM_STAMPED_HPP

#include <string>
#include "tf2/time.hpp"
#include "tf2/msg/transform.hpp"
namespace tf2{
namespace msg{
	struct TransformStamped {
		// EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		// === Header ===
		uint32_t seq = 0;  // Optional: only needed if you care about message order
		tf2::TimePoint timestamp;
		std::string frame_id;

		// === Body ===
		std::string child_frame_id;

		tf2::msg::Transform transform;

		TransformStamped()
  		: transform() { std::cout << "Constructed TransformStamped  at " << this << std::endl; }

		// Eigen::Vector3d translation;
		// Eigen::Quaterniond rotation;

		// TransformStamped()
		// 	: translation(Eigen::Vector3d::Zero()),
		// 	rotation(Eigen::Quaterniond::Identity()) {}
	};
}
}

#endif