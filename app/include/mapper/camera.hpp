#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <iostream>

// Simpel kamera defintion
// Updatera detta med mer relevant information, som projektet går framåt
struct CameraInfo {
    // Ska kamera matrisen vara från Eigen eller openCV?
    Eigen::Matrix3d K;  // Camera matrix
    Eigen::Matrix3d R;  // Rotation matrix
    Eigen::Vector3d T;  // Translation matrix
    Eigen::Matrix<double, 3, 4> P;  // Projection matrix, K*[R|T]. Behövs denna?
    std::vector<double> distortion;  // Distortion coefficients
    const unsigned int width = 100; // Ändra till deras korrekta värden
    const unsigned int height = 100; // Detta kanske man kan göra när man updaterar med kalibreringsdata
};

// Kanske ska vara i en annan fil?
typedef Eigen::Vector2d Point;
typedef Eigen::Vector3d Keypoint;

class PinholeCameraModel {
private:
    CameraInfo camera_info;
    void readYAML(const std::string& filename, CameraInfo &camera);

public:
    PinholeCameraModel();
    PinholeCameraModel(const std::string& filename, CameraInfo &camera);
    // ~PinholeCameraModel();

    // Project a 3D point to a 2D point
    Keypoint projectPoint(const Point& point3d);

    // Back-project a 2D point to a 3D point
    Eigen::Vector3d backProjectPoint(const Keypoint& point2d);
};

#endif