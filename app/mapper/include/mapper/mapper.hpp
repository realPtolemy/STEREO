#ifndef MAPPER_HPP
#define MAPPER_HPP

#include "mapper/camera.hpp"
#include "event.hpp"
#include "mapper/mapper_emvs_stereo.hpp"
#include "mapper/transformation.hpp"
#include "tf2/time.hpp"

#include <iostream>
#include <vector>
#include <fstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <map>
#include <deque>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp> // For image display
#include <opencv2/imgproc.hpp> // For image processing

class Mapper
{
private:
    // online data
    std::deque<std::vector<Event>> events_left_, events_right_;
    // , events_tri_;
    // Fixa Time klassen
    std::map<tf2::TimePoint, Transformation> poses_;

    // result
    EMVS::PointCloud::Ptr pc_, pc_global_;
    cv::Mat depth_map, confidence_map, confidence_map_0, confidence_map_1, confidence_map_2, semidense_mask;

    // Disparity Space Image (DSI) parameters. Section 5.2 in the IJCV paper.
    double min_depth = 0.3;
    double max_depth = 5.0;
    double fov_deg = 0.0;
    int dimX = 0;
    int dimY = 0;
    int dimZ = 100;

    // Depth map parameters (selection and noise removal). Section 5.2.3 in the IJCV paper.
    int adaptive_threshold_kernel_size = 5;
    double adaptive_threshold_c = 5.0;
    int median_filter_size = 5;

    // Point cloud parameters (noise removal). Section 5.2.4 in the IJCV paper.
    double radius_search = 0.05;
    // int min_num_negihbors = 3;

    // Process parameters
    const std::string m_calib_file_cam1 =
        "calibration/calibrationData/calibration_data_camera_0.yaml";
    const std::string m_calib_file_cam2 =
        "calibration/calibrationData/calibration_data_camera_1.yaml";
    // std::string calib_path, mocap_calib_path;
    // std::string calib_type;
    int process_method = 1;
    int num_intervals = 4;
    int stereo_fusion = 2;
    int temporal_fusion = 4;

    // std::condition_variable m_local_dsi_cv;
    // std::condition_variable m_merge_dsi_cv;
    std::mutex data_mutex_;
    // std::mutex m_merge_dsi_mutex;
    // int m_threads = 0;
    // bool m_merge_ready = true;
    const int m_NUM_OF_CAMERAS = 2;

    // calibration params
    Eigen::Matrix4d mat4_1_0, mat4_2_0, mat4_hand_eye;
    CameraInfo camera1, camera2;
    PinholeCameraModel cam0, cam1, cam2;


    // EMVS::OptionsDepthMap opts_depth_map;
    // EMVS::ShapeDSI dsi_shape;

    // ros::Time current_ts_;
    tf2::TimePoint current_ts_;
    tf2::TimePoint first_ev_ts;

    // int beg_ev_left_idx;
    // int beg_ev_right_idx;
    // int beg_ev_tri_idx;
    // ros::Time beg_ev_left_ts;
    // ros::Time beg_ev_right_ts;
    // ros::Time beg_ev_tri_ts;
    // ros::Time first_ev_ts;

    bool initialized_;
    bool initialized_tf_;
    bool auto_trigger_;
    bool on_demand_;
    bool accumulate_pc_;

    int NUM_EV_PER_MAP = 512;
    Transformation T_rv_w_;

    cv::Mat event_image0_, event_image1_, event_image_live_;

    tf2::TimePoint latest_tf_stamp_;

    enum MapperState {MAPPING, IDLE};
    MapperState state_;

    float max_duration_, min_duration_, init_wait_t_;
    bool auto_copilot_;

    // #define dimX 0
    // #define dimY 0
    // #define dimZ 100
    // #define for_deg 0.0
    // #define min_depth 0.3
    // #define max_depth 5.0
    // #define NUM_EVENTS 512

    Event parse_line(const std::string& line);
    void camera_thread(const std::string &event_file_path, std::vector<Event> &camera1_events, std::deque<std::vector<Event>> &event_queue);
    void checkEventQueue(std::deque<std::vector<Event>> &event_queue);
    void dsi_merger(std::vector<Event> &camera1_events, std::vector<Event> &camera2_events);
    void mappingLoop();
public:
    Mapper();
};

#endif