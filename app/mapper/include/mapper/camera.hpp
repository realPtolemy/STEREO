#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <iostream>

struct CameraInfo {
    // Intrinsic parameters
    cv::Matx33d K_cv;                      // OpenCV camera matrix
    Eigen::Matrix3d K_eigen;               // Eigen camera matrix
    Eigen::Matrix3d Kinv_eigen;            // Inverse intrinsic matrix (Eigen)
    double fx_, fy_, cx_, cy_;
    // Extrinsics
    cv::Matx33d R_cv = cv::Matx33d::eye(); // OpenCV rotation matrix
    cv::Vec3d T_cv = cv::Vec3d(0, 0, 0);   // OpenCV translation vector

    Eigen::Matrix3d R_eigen = Eigen::Matrix3d::Identity();
    Eigen::Vector3d T_eigen = Eigen::Vector3d::Zero();

    // Projection matrix
    cv::Matx34d P_cv = cv::Matx34d::zeros();

    // Distortion
    std::vector<double> distortion;
    std::string distortion_model = "fish_eye";

    // Resolution
    unsigned int width = 100;
    unsigned int height = 100;
};


// Kanske ska vara i en annan fil?
typedef Eigen::Vector3d Point;
typedef Eigen::Vector2d Keypoint;
typedef Eigen::Vector3d BearingVector;

class PinholeCameraModel {
private:
    void readYAML(const std::string& filename, CameraInfo& camera);

    CameraInfo camera_info;

    Eigen::Matrix3d toEigen(const cv::Matx33d& m) const;
    cv::Matx33d toCv(const Eigen::Matrix3d& m) const;

public:
    PinholeCameraModel();
    PinholeCameraModel(int width, int height, double fx, double fy, double cx, double cy);
    PinholeCameraModel(const std::string& filename, CameraInfo& camera);

    /**
     * Projects a 3d point (in camera frame) to the pixel coordinates
     *
     * @param P 3d point in camera frame
     *
     * @return Pixel coordinates of the projected point
     */
    inline Keypoint project3dToPixel(const Point& P) {
        return Keypoint(
            camera_info.fx_ * P[0] / P[2] + camera_info.cx_,
            camera_info.fy_ * P[1] / P[2] + camera_info.cy_
        );
    }

    /**
     * Backprojects the pixel coordinates to 3d ray
     *
     * @param u Keypoint representing the pixel coordinates
     *
     * @return Bearing vector being the 3d ray, backprojection of P
     */
    inline BearingVector projectPixelTo3dRay(const Keypoint& u) {
        return BearingVector(
            (u[0] - camera_info.cx_) / camera_info.fx_,
            (u[1] - camera_info.cy_) / camera_info.fy_,
            1.0
        );
    }

    // Accessors
    cv::Size fullResolution() const;

    double fx() const { return camera_info.fx_; }
    double fy() const { return camera_info.fy_; }
    double cx() const { return camera_info.cx_; }
    double cy() const { return camera_info.cy_; }

    Eigen::Matrix3d getKinv() const { return camera_info.Kinv_eigen; }
    Eigen::Matrix3d getK_eigen() const { return camera_info.K_eigen; }
    cv::Matx33d intrinsicMatrix() const { return camera_info.K_cv; }

    cv::Matx34d getProjectionMatrix() const { return camera_info.P_cv; }
    cv::Mat distortionCoeffs() const { return cv::Mat(camera_info.distortion); }
    std::string distortionModel() const { return camera_info.distortion_model; }

    cv::Matx33d rotationMatrix() const { return camera_info.R_cv; }
    Eigen::Matrix3d rotationMatrixEigen() const { return camera_info.R_eigen; }

    cv::Vec3d translationCV() const { return camera_info.T_cv; }
    Eigen::Vector3d translationEigen() const { return camera_info.T_eigen; }

};

#endif