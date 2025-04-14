#ifndef TF2_MSG_TRANSFORM_HPP
#define TF2_MSG_TRANSFORM_HPP

// #include <tf2/msg/transform_stamped.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tf2{
namespace msg{
	struct Transform {
		// EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Eigen::Vector3d translation;
		Eigen::Quaterniond rotation;

		Transform()
			: translation(Eigen::Vector3d::Zero()),
			  rotation(Eigen::Quaterniond::Identity()) {}
	};
}
}


#endif