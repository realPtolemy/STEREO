#include <iostream>
#include <vector>
#include <chrono>
#include <cstdint>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>  // For image display
#include <opencv2/imgproc.hpp>  // For image processing

#include "Camera.hpp"
#include "Event.hpp"
#include "YAMLReader.hpp"

// void test(){
//   cv::FileStorage fs("calibration_data.yaml", cv::FileStorage::READ);
//   cv::Mat cameraMatrix, distCoeffs, undistortMap_x, undistortMap_y;
//   fs["cameraMatrix"] >> cameraMatrix;
//   fs["distCoeffs"] >> distCoeffs;
//   fs["undistortMap_x"] >> undistortMap_x;
//   fs["undistortMap_y"] >> undistortMap_y;
//   fs.release();

//   // Now use these maps with cv::remap
//   cv::Mat inputImage = cv::imread("your_image.png");
//   cv::Mat undistortedImage;
//   cv::remap(inputImage, undistortedImage, undistortMap_x, undistortMap_y, cv::INTER_LINEAR);
// }

using Eigen::MatrixXd;

const std::string calib_file_cam1 = "../calibration/calibration_data_cam1.yaml";

int main(int argc, char** argv)
{
  // Parse input arguemnts?

  // Create a camera object
  CameraInfo camera1;
  readYAML(calib_file_cam1);
  // Här måste vi fylla ut kamera objektet med kalibreingsdata
  //PinholeCameraModel camera;

  Eigen::Matrix4d mat4_1_0, mat4_2_0, mat4_hand_eye;

  std::vector<event> camera1_events, camera2_events;
  //std::vector<u_int64_t, > poses;
  // Define/Load camera calibration parameters

  // Create a UDP or file reader for event data. Multi threaded?

  // Initialize the DSI

  // Read events, user buffers?

}