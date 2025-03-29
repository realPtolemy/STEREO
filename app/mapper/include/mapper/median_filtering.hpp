#ifndef MEDIAN_FILTERING_HPP
#define MEDIAN_FILTERING_HPP

#include <opencv2/core.hpp>

void huangMedianFilter(const cv::Mat& img, cv::Mat& out_img, const cv::Mat& mask, int patch_size);

#endif