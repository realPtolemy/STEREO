#ifndef MAPPER_HPP
#define MAPPER_HPP

#include "mapper/camera.hpp"
#include "event.hpp"
#include "mapper/mapper_emvs_stereo.hpp"

#include <iostream>
#include <vector>
#include <fstream>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp> // For image display
#include <opencv2/imgproc.hpp> // For image processing

class Mapper
{
private:
    const std::string m_calib_file_cam1 =
        "calibration/calibrationData/calibration_data_camera_0.yaml";
    const std::string m_calib_file_cam2 =
        "calibration/calibrationData/calibration_data_camera_1.yaml";

    std::condition_variable m_local_dsi_cv;
    std::condition_variable m_merge_dsi_cv;
    std::mutex m_local_dsi_mutex;
    std::mutex m_merge_dsi_mutex;
    int m_threads = 0;
    bool m_merge_ready = true;
    const int m_NUM_OF_CAMERAS = 2;
    #define dimX 0
    #define dimY 0
    #define dimZ 100
    #define for_deg 0.0
    #define min_depth 0.3
    #define max_depth 5.0
    #define NUM_EVENTS 512

    event parse_line(const std::string& line);
    void local_dsi_csv(const std::string &event_file_path, std::vector<event> &camera1_events, int id);
    void dsi_merger(std::vector<event> &camera1_events, std::vector<event> &camera2_events);
public:
    Mapper();
};

#endif