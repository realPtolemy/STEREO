#ifndef YAMLREADER_HPP
#define YAMLREADER_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <iostream>

using namespace std;
// Orkar inte göra det i en cpp fil just nu
inline void readYAML(const std::string& filename) {
    cv::FileStorage fs (filename, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        std::cerr << "Error: Could not open file: " << filename << std::endl;
        return;
    }

    cv::Mat cameraMatrix, distCoeffs;
        fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();

    std::cout << "Camera Matrix:\n" << cameraMatrix << std::endl;
    std::cout << "Distortion Coefficients:\n" << distCoeffs << std::endl;
}


#endif