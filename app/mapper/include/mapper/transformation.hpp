#ifndef TRANSFORMATION_HPP
#define TRANSFORMATION_HPP

#include <kindr/poses/HomogeneousTransformation.hpp>
#include <kindr/rotations/RotationQuaternion.hpp>
#include <kindr/phys_quant/PhysicalQuantities.hpp>

class Transformation
{
    using Position3D = kindr::Position<double, 3>;
    using RotationD = kindr::RotationQuaternion<double>;
    using TransformationType = kindr::HomogeneousTransformation<double,Position3D,RotationD>;
public:


    Transformation() : T_(TransformationType::Identity()) {}
    Transformation(Position3D position, RotationD rotation) : T_(position, rotation) {}
    ~Transformation() = default;

    inline Eigen::Vector3d getPosition() const {
        return T_.getPosition().toImplementation();
    }

    inline kindr::RotationQuaternion<double> getRotation() const {
        return T_.getRotation();
    }

    inline Eigen::Matrix3d getRotationMatrix() const {
        return T_.getRotation().toImplementation().toRotationMatrix();
    }

    inline Transformation inverse() const {
        auto R_inv = T_.getRotation().inverted();
        auto t_inv = R_inv.rotate(-T_.getPosition());
        Transformation result;
        result.T_ = TransformationType(t_inv, R_inv);
        return result;
    }

    inline Eigen::Matrix4d getTransformationMatrix(){
        return T_.getTransformationMatrix();
    }

    inline Transformation operator*(const Transformation& other) const {
        Transformation result;
        result.T_ = T_ * other.T_;
        return result;
    }

private:
    TransformationType T_;
};

#endif