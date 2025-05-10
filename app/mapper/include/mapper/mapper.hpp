#ifndef MAPPER_HPP
#define MAPPER_HPP

#include "app/camera.hpp"
#include "app/event.hpp"
#include "mapper/mapper_emvs_stereo.hpp"
#include "mapper/transformation.hpp"
#include "mapper/pointcloud_processing.hpp"
#include "app/shared_state.hpp"
#include "tf2/time.hpp"
#include "tf2/LinearMath/tf2_eigen.hpp"
#include "mapper/process1_live.hpp"

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

/*
    This code is based upon the mapper.hpp file from ES-PTAM
    Though with some modifications, mainly ragrding tf and ros::time
*/

class Mapper
{
private:
    // Shared state data
    SharedState *shared_state_;

    std::atomic<bool> running_{true};  // Control flag
    std::thread mapper_thread_;

    std::shared_ptr<tf2::BufferCore> tf_;
    std::string world_frame_id_;
    std::string frame_id_;
    std::string regular_frame_id_;
    std::string bootstrap_frame_id_;
    std::string out_path;
    // online data
    // std::deque<std::vector<Event>> events_left_, events_right_;
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
    int dimZ = 256;
    int max_confidence = 0;
    bool full_seq = false;
    bool save_conf_stats = false;
    bool save_mono = false;
    double rv_pos = 0;
    int min_num_neighbors = 3;

    // Depth map parameters (selection and noise removal). Section 5.2.3 in the IJCV paper.
    int adaptive_threshold_kernel_size = 5;
    double adaptive_threshold_c = 5.0;
    int median_filter_size = 5;

    // Point cloud parameters (noise removal). Section 5.2.4 in the IJCV paper.
    double radius_search = 0.05;
    // int min_num_negihbors = 3;

    // Process parameters

    // std::string calib_path, mocap_calib_path;
    // std::string calib_type;
    int process_method = 1;
    int num_intervals = 4;
    int stereo_fusion = 2;
    int temporal_fusion = 4;

    std::mutex data_mutex_;
    const int NUM_OF_CAMERAS_ = 2;

    // calibration params
    Eigen::Matrix4d mat4_1_0, mat4_2_0, mat4_hand_eye;
    // CameraInfo camera1, camera2;
    PinholeCameraModel cam0, cam1, cam2;


    EMVS::OptionsDepthMap opts_depth_map;
    EMVS::ShapeDSI dsi_shape;

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

    int EVENT_BATCH_SIZE = 1024;
    int NUM_EV_PER_MAP = 100000;
    Transformation T_rv_w_;

    cv::Mat event_image0_, event_image1_, event_image_live_;

    tf2::TimePoint latest_tf_stamp_;

    enum MapperState {MAPPING, IDLE};
    MapperState state_;

    float max_duration_, min_duration_, init_wait_t_;
    bool auto_copilot_;

    /**
     * Reads line that has the structure of an event, then parses it into an Event object.
     *
     * @param line The line to parse.
     * @return The parsed Event object.
     */
    Event parse_line(const std::string& line);
    /**
     * A thread that reads events from a file, parses them, and pushes them into a queue.
     *
     * @param event_file_path The path to the event file.
     * @param camera_events A vector to store the parsed events.
     * @param event_queue A queue that stores batched events.
     */
    void camera_thread_csv(const std::string &event_file_path, std::vector<Event> &camera1_events, EventQueue<Event> &event_queue);
    void camera_thread_udp(Server& server, std::vector<Event> &camera_events, EventQueue<Event> &event_queue);
    Event parse_bufferd_data(std::string &buffered_data);
    void mappingLoop();
    void MappingAtTime(
        tf2::TimePoint current_ts,
        std::vector<Event> &events_left_,
        std::vector<Event> &events_right_,
        std::vector<Event> &events_tri_,
        std::string frame_id
    );
    void publishMsgs(std::string frame_id);
    void publishImgs(std::string frame_id);
    bool waitForTransformSimple(
        const std::shared_ptr<tf2::BufferCore> & buffer,
        const std::string & target_frame,
        const std::string & source_frame,
        const tf2::TimePoint & time,
        const tf2::Duration & timeout,
        const tf2::Duration & polling_sleep = std::chrono::milliseconds(10)
    );
    void tfCallback();
public:
    Mapper(SharedState &shared_state);
    void mapperRun();
    ~Mapper();
};

#endif