#include "mapper/camera.hpp"
#include "iostream"
#include <filesystem>
PinholeCameraModel::PinholeCameraModel()
{
}

PinholeCameraModel::PinholeCameraModel(const std::string& filename, CameraInfo &camera)
    : camera_info(camera)
{
    readYAML(filename, camera);
}

void PinholeCameraModel::readYAML(const std::string& filename, CameraInfo &camera)
{

    // std::cout << "Current Working Directory: "
            //   << std::filesystem::current_path() << std::endl;

    cv::FileStorage fs (filename, cv::FileStorage::READ);

    cv::Mat cameraMatrix, distCoeffs;
        fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();

    Eigen::Matrix3d tmp = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(cameraMatrix.ptr<double>());
    camera.K = tmp;
    camera.distortion = {
        distCoeffs.at<double>(0),
        distCoeffs.at<double>(1),
        distCoeffs.at<double>(2),
        distCoeffs.at<double>(3),
        distCoeffs.at<double>(4)
    };

    // Inital values, these are updated with the help of the tracking module!
    camera.R = Eigen::Matrix3d::Identity();
    camera.T = Eigen::Vector3d::Zero();
    camera.P = Eigen::Matrix<double, 3, 4>::Zero();
    camera.P.block<3,3>(0,0) = camera.K;  // Top-left 3x3 block is K
    camera.P.block<3,1>(0,3) = camera.K * camera.T; // Last column is K*T, which is zero here
}