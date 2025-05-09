#include "app/camera.hpp"
#include "iostream"
#include <filesystem>

PinholeCameraModel::PinholeCameraModel() {}

PinholeCameraModel::PinholeCameraModel(int width, int height, double fx, double fy, double cx, double cy)
{
    camera_info.fx_ = fx;
    camera_info.fy_ = fy;
    camera_info.cx_ = cx;
    camera_info.cy_ = cy;

    camera_info.width = width;
    camera_info.height = height;

    camera_info.K_cv = cv::Matx33d(fx, 0, cx,
                                   0, fy, cy,
                                   0,  0,  1);

    camera_info.K_eigen = toEigen(camera_info.K_cv);
    camera_info.Kinv_eigen = camera_info.K_eigen.inverse();
}

PinholeCameraModel::PinholeCameraModel(const std::string& filename)
{
    readYAML(filename);
}

void PinholeCameraModel::readYAML(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    cv::Mat K, distCoeffs;

    if (!fs["camera_matrix"].empty() && !fs["distortion_coefficients"].empty()) {
        fs["camera_matrix"] >> K;
        fs["distortion_coefficients"] >> distCoeffs;

        camera_info.K_cv = K;
        camera_info.K_eigen = toEigen(camera_info.K_cv);
        camera_info.Kinv_eigen = camera_info.K_eigen.inverse();

        for (int i = 0; i < distCoeffs.total(); ++i) {
            camera_info.distortion.push_back(distCoeffs.at<double>(i));
        }

        camera_info.fx_ = camera_info.K_cv(0, 0);
        camera_info.fy_ = camera_info.K_cv(1, 1);
        camera_info.cx_ = camera_info.K_cv(0, 2);
        camera_info.cy_ = camera_info.K_cv(1, 2);
        camera_info.newcameramtx = cv::getOptimalNewCameraMatrix(K, distCoeffs, cv::Size(camera_info.width, camera_info.height), 1, cv::Size(camera_info.width, camera_info.height));


        // // Fill projection matrix
        // cv::FileNode rotVecsNode = fs["rotationVectors"];
        // cv::FileNode transVecsNode = fs["translationVectors"];

        // cv::Mat rvec;
        // rotVecsNode[0] >> rvec;
        // cv::Rodrigues(rvec, camera_info.R_cv);

        // cv::Mat tvec;
        // transVecsNode[0] >> tvec;
        // camera_info.T_cv = cv::Vec3d(tvec);

        // cv::Mat Rt;
        // cv::hconcat(camera_info.R_cv, cv::Mat(camera_info.T_cv), Rt);
        // cv::Mat P = camera_info.K_cv * Rt;
        // P.copyTo(camera_info.P_cv);

        // camera_info.R_cv = P(cv::Rect(0,0,3,3));


        // Hand eye matrix
        // Eigen::Matrix3d R_eigen;
        // Eigen::Vector3d t_eigen;

        // cv::cv2eigen(camera_info.R_cv, R_eigen);
        // t_eigen = Eigen::Vector3d(camera_info.T_cv[0], camera_info.T_cv[1], camera_info.T_cv[2]);

        // hand_eye_.setIdentity();
        // hand_eye_.block<3, 3>(0, 0) = R_eigen;
        // hand_eye_.block<3, 1>(0, 3) = t_eigen;

    } else {
        std::cerr << "Invalid camera calibration file: missing keys!" << std::endl;
    }
    fs.release();
}

// void PinholeCameraModel::readStereoCalib(const std::string& filename, PinholeCameraModel& other_camera){
void PinholeCameraModel::readStereoCalib(const std::string& filename, Eigen::Matrix4d& mat4_1_0){
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs["rotation_matrix"].empty() && !fs["translation_vector"].empty()) {
        cv::Mat R_lr, T_lr;
        fs["rotation_matrix"] >> R_lr;
        fs["translation_vector"] >> T_lr;

        T_lr /= 1000;

        std::cout << "#1 R_lr:\n" << R_lr << "\n#1 T_lr:\n" << T_lr << std::endl;


        // Invert to get right→left:
        cv::Mat R_rl = R_lr.t();
        cv::Mat T_rl = -R_rl * T_lr;


        // 3x4 [R|T]
        // cv::Mat ext_rl(3, 4, CV_64F);
        // R_rl.copyTo(ext_rl(cv::Rect(0,0,3,3)));
        // T_rl.copyTo(ext_rl(cv::Rect(3,0,1,3)));
        // cv::cv2eigen(ext_rl, mat4_1_0);

        // // 4x4 Homogenous matrix
         cv::Mat H_rl = cv::Mat::eye(4, 4, CV_64F);
         R_rl.copyTo(H_rl(cv::Rect(0,0,3,3)));
         T_rl.copyTo(H_rl(cv::Rect(3,0,1,3)));
         cv::cv2eigen(H_rl, mat4_1_0);

         std::cout << "#2 R_rl:\n" << R_rl << "\n#2 T_rl:\n" << T_rl << "\nmat4_1_0:\n" << mat4_1_0 << std::endl;
    }
    fs.release();
}

// Convert OpenCV matrix to Eigen
Eigen::Matrix3d PinholeCameraModel::toEigen(const cv::Matx33d& m) const
{
    Eigen::Matrix3d out;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out(r, c) = m(r, c);
    return out;
}

cv::Size PinholeCameraModel::fullResolution() const
{
    return cv::Size(camera_info.width, camera_info.height);
}

cv::Point PinholeCameraModel::rectifyPoint(cv::Point point)
{
    cv::Point2f raw32 = point, rect32;
    const cv::Mat src_pt(1, 1, CV_32FC2, &raw32.x);
    cv::Mat dst_pt(1, 1, CV_32FC2, &rect32.x);
    cv::undistortPoints(src_pt, dst_pt, camera_info.K_cv, cv::Mat(camera_info.distortion),cv::noArray() , camera_info.newcameramtx);
    return rect32;
}
void PinholeCameraModel::loadCamerInfo(CameraInfo &info)
{
    camera_info = info;

    camera_info.K_eigen = toEigen(camera_info.K_cv);
    camera_info.Kinv_eigen = camera_info.K_eigen.inverse();

    camera_info.fx_ = camera_info.K_cv(0, 0);
    camera_info.fy_ = camera_info.K_cv(1, 1);
    camera_info.cx_ = camera_info.K_cv(0, 2);
    camera_info.cy_ = camera_info.K_cv(1, 2);

    camera_info.P_cv = cv::Matx34d::zeros();
    camera_info.P_cv.get_minor<3, 3>(0, 0) = camera_info.K_cv;
}