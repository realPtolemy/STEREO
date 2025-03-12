#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <vector>
#include <Eigen/Dense>

// Simpel kamera defintion
// Updatera detta med mer relevant information, som projektet går framåt
struct CameraInfo {
    double fx, fy;  // Focal lengths
    double cx, cy;  // Principal point
    std::vector<double> distortion;  // Distortion coefficients
    // Kommer behöva, K matrix, R matrix, T matrix and maybe some other stuff

    const unsigned int width = 100; // Ändra till deras korrekta värden
    const unsigned int height = 100; // Detta kanske man kan göra när man updaterar med kalibreringsdata
};

// Kanske ska vara i en annan fil?
typedef Eigen::Vector2d Point;
typedef Eigen::Vector3d Keypoint;

class PinholeCameraModel {
private:
    CameraInfo camera_info;
public:
    PinholeCameraModel();
    PinholeCameraModel(const CameraInfo& camera_info);
    ~PinholeCameraModel();

    // Project a 3D point to a 2D point
    Keypoint projectPoint(const Point& point3d);

    // Back-project a 2D point to a 3D point
    Eigen::Vector3d backProjectPoint(const Keypoint& point2d);
};

#endif