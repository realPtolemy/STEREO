#include <iostream>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>  // For image display
#include <opencv2/imgproc.hpp>  // For image processing
#include "camera.hpp"

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

int main(int argc, char** argv)
{
  // Parse input arguemnts?

  // Create a camera object
  CameraInfo camera1;
  std::cout << "Camera1 width: " << camera1.width << std::endl;
  // Define/Load camera calibration parameters

  // Create a UDP or file reader for event data. Multi threaded?

  // Initialize the DSI

  // Read events, user buffers?

}