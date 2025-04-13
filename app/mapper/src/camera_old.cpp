// #include "mapper/camera.hpp"
// #include "iostream"
// #include <filesystem>

// PinholeCameraModel::PinholeCameraModel() {}

// PinholeCameraModel::PinholeCameraModel(int width, int height, double fx, double fy, double cx, double cy) {
//     camera_info.fx_ = fx;
//     camera_info.fy_ = fy;
//     camera_info.cx_ = cx;
//     camera_info.cy_ = cy;

//     camera_info.width = width;
//     camera_info.height = height;

//     camera_info.K_cv = cv::Matx33d(fx, 0, cx,
//                                    0, fy, cy,
//                                    0,  0,  1);

//     camera_info.K_eigen = toEigen(camera_info.K_cv);
//     camera_info.Kinv_eigen = camera_info.K_eigen.inverse();
// }

// PinholeCameraModel::PinholeCameraModel(const std::string& filename, CameraInfo &camera)
//     : camera_info(camera)
// {
//     readYAML(filename, camera);
// }

// void PinholeCameraModel::readYAML(const std::string& filename, CameraInfo& camera) {
//     cv::FileStorage fs(filename, cv::FileStorage::READ);
//     cv::Mat K, distCoeffs;

//     fs["cameraMatrix"] >> K;
//     fs["distCoeffs"] >> distCoeffs;
//     fs.release();

//     K.copyTo(cv::Mat(camera.K_cv));
//     camera.K_eigen = toEigen(camera.K_cv);
//     camera.Kinv_eigen = camera.K_eigen.inverse();

//     for (int i = 0; i < distCoeffs.total(); ++i) {
//         camera.distortion.push_back(distCoeffs.at<double>(i));
//     }

//     // Fill projection matrix
//     camera.R_cv = cv::Matx33d::eye();
//     camera.T_cv = cv::Vec3d(0, 0, 0);
//     camera.R_eigen = Eigen::Matrix3d::Identity();
//     camera.T_eigen = Eigen::Vector3d::Zero();

//     camera.P_cv = cv::Matx34d::zeros();
//     camera.P_cv.get_minor<3, 3>(0, 0) = camera.K_cv;

//     camera_info.fx_ = camera.K_cv(0, 0);
//     camera_info.fy_ = camera.K_cv(1, 1);
//     camera_info.cx_ = camera.K_cv(0, 2);
//     camera_info.cy_ = camera.K_cv(1, 2);
// }

// // Convert OpenCV matrix to Eigen
// Eigen::Matrix3d PinholeCameraModel::toEigen(const cv::Matx33d& m) const {
//     Eigen::Matrix3d out;
//     for (int r = 0; r < 3; ++r)
//         for (int c = 0; c < 3; ++c)
//             out(r, c) = m(r, c);
//     return out;
// }

// // // Convert Eigen matrix to OpenCV
// // cv::Matx33d PinholeCameraModel::toCv(const Eigen::Matrix3d& m) const {
// //     cv::Matx33d out;
// //     for (int r = 0; r < 3; ++r)
// //         for (int c = 0; c < 3; ++c)
// //             out(r, c) = m(r, c);
// //     return out;
// // }

// cv::Size PinholeCameraModel::fullResolution() const
// {
//     return cv::Size(camera_info.width, camera_info.height);
// }

// cv::Point PinholeCameraModel::rectifyPoint(cv::Point point)
// {

//     cv::Point2f raw32 = point, rect32;
//     const cv::Mat src_pt(1, 1, CV_32FC2, &raw32.x);
//     cv::Mat dst_pt(1, 1, CV_32FC2, &rect32.x);
//     cv::undistortPoints(src_pt, dst_pt, camera_info.K_cv, cv::Mat(camera_info.distortion), camera_info.R_cv, camera_info.P_cv);
//     return rect32;
//     return cv::Point();
// }
// void PinholeCameraModel::loadCamerInfo(CameraInfo &info)
// {
//     camera_info.width = info.width;
//     camera_info.height = info.height;
//     // Fyll ut denna klass
// }