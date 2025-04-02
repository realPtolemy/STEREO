#ifndef TRANSFORMATION_HPP
#define TRANSFORMATION_HPP

// #include <kindr/poses/HomogeneousTransformation.hpp>
// #include <kindr/rotations/RotationQuaternion.hpp>
// #include <kindr/phys_quant/PhysicalQuantities.hpp>

#include <kindr/minimal/quat-transformation.h>

using Transformation = kindr::minimal::QuatTransformation;
using Quaternion = kindr::minimal::RotationQuaternion;

// class Transformation
// {
//     using Position3D = kindr::Position<double, 3>;
//     using RotationD = kindr::RotationQuaternion<double>;
//     using TransformationType = kindr::HomogeneousTransformation<double,Position3D,RotationD>;
// public:

//     Transformation() : T_(TransformationType::Identity()) {}
//     Transformation(Position3D position, RotationD rotation) : T_(position, rotation) {}
//     Transformation(const Eigen::Matrix4d& matrix) {
//         Eigen::Matrix3d rot = matrix.block<3,3>(0,0);
//         Eigen::Vector3d pos = matrix.block<3,1>(0,3);

//         Eigen::Quaterniond q_eigen(rot);        // Matrix → Eigen quaternion
//         RotationD rotation(q_eigen);            // Eigen quaternion → Kindr RotationD
//         Position3D position(pos);

//         T_ = TransformationType(position, rotation);
//     }
//     ~Transformation() = default;

//     inline Eigen::Vector3d getPosition() const {
//         return T_.getPosition().toImplementation();
//     }

//     inline kindr::RotationQuaternion<double> getRotation() const {
//         return T_.getRotation();
//     }

//     inline Eigen::Matrix3d getRotationMatrix() const {
//         return T_.getRotation().toImplementation().toRotationMatrix();
//     }

//     inline Transformation inverse() const {
//         auto R_inv = T_.getRotation().inverted();
//         auto t_inv = R_inv.rotate(-T_.getPosition());
//         Transformation result;
//         result.T_ = TransformationType(t_inv, R_inv);
//         return result;
//     }

//     inline Eigen::Matrix4d getTransformationMatrix(){
//         return T_.getTransformationMatrix();
//     }

//     inline Transformation operator*(const Transformation& other) const {
//         Transformation result;
//         std::cout << "T1:\n" << this->T_.getTransformationMatrix() << std::endl;
//         std::cout << "T2:\n" << other.T_.getTransformationMatrix() << std::endl;


//         std::cout << "Result:\n" << (this->T_.operator*(other.T_)).getTransformationMatrix() << std::endl;
//         return result;
//     }


// private:
//     TransformationType T_;
// };

#endif