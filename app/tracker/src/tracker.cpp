/**
 *  Code based upon the work from ES-PTAM : https://github.com/tub-rip/ES-PTAM
 *  We have just modified this code to work without ROS, we have not written the core
 *  logic of the code and do not claim ownership of it.
 *
 *  If any function is written by us, then we have mentioned it in comments.
 *
 */

#include "tracker/tracker.hpp"
#include <kindr/minimal/quat-transformation.h>
#include <pcl/conversions.h>

#include <cstddef>  // For offsetof

#include "tf2/LinearMath/tf2_eigen.hpp"

#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <sophus/se3.hpp>

// #define TRACKING_PERF
using Transformation = kindr::minimal::QuatTransformation;
using Quaternion = kindr::minimal::RotationQuaternion;

void Tracker::postCameraLoaded() {
    width_ = c_.fullResolution().width;
    height_ = c_.fullResolution().height;
    fx_ = c_.fx();
    fy_ = c_.fy();
    cx_ = c_.cx();
    cy_ = c_.cy();
    rect_ = cv::Rect(0, 0, width_, height_);

    // LOG(INFO) << "----------------------------" << CV_VERSION;
    float fov = 2. * std::atan(c_.fullResolution().width / 2. / c_.fx());
    // LOG(INFO) << "Field of view: " << fov / M_PI * 180.;

    new_img_ = cv::Mat(c_.fullResolution(), CV_32F, cv::Scalar(0));

    CameraInfo cam_ref = c_.getCameraInfo();
    cam_ref.width = 680;
    cam_ref.height = 480;

    cam_ref.cx_ = 0.5 * (double)cam_ref.width;
    cam_ref.cy_ = 0.5 * (double)cam_ref.height;

    // Set K matrix (3x3)
    cam_ref.K_cv(0, 2) = cam_ref.cx_;
    cam_ref.K_cv(1, 2) = cam_ref.cy_;

    // Determine focal length
    double f_ref = 0.0;
    if (f_ref == 0.0)
        f_ref = c_.fx();
    else {
        double f_ref_rad = f_ref * CV_PI / 180.0;
        f_ref = 0.5 * cam_ref.width / std::tan(0.5 * f_ref_rad);
    }

    cam_ref.fx_ = f_ref;
    cam_ref.fy_ = f_ref;

    // Fill in the intrinsics
    cam_ref.K_cv(0, 0) = cam_ref.fx_;
    cam_ref.K_cv(1, 1) = cam_ref.fy_;
    cam_ref.K_cv(0, 1) = 0.0;
    cam_ref.K_cv(1, 0) = 0.0;
    cam_ref.K_cv(2, 0) = 0.0;
    cam_ref.K_cv(2, 1) = 0.0;
    cam_ref.K_cv(2, 2) = 1.0;

    // Also fill in projection matrix (3x4)
    cam_ref.P_cv(0, 0) = cam_ref.fx_;
    cam_ref.P_cv(0, 2) = cam_ref.cx_;
    cam_ref.P_cv(1, 1) = cam_ref.fy_;
    cam_ref.P_cv(1, 2) = cam_ref.cy_;
    cam_ref.P_cv(2, 2) = 1.0;

    c_ref_ = c_;
    reset();
}

Tracker::Tracker(SharedState &shared_state):
    shared_state_(&shared_state),
    tf_(shared_state_->tf_),
    cur_ev_(0),
    kf_ev_(0),
    noise_rate_(10000),
    frame_size_(2500),
    step_size_(2500),
    idle_(true)
{
    batch_size_ = 500;
    max_iterations_ = 100;
    map_blur_ = 5;

    pyramid_levels_ = 1;

    weight_scale_trans_ = 0.;
    weight_scale_rot_ = 0.;
    first_event_ = false;

    T_world_kf_ = T_kf_ref_ = T_ref_cam_ = T_cur_ref_ =
        Eigen::Affine3f::Identity();

    map_ = PointCloud::Ptr(new PointCloud);
    map_local_ = PointCloud::Ptr(new PointCloud);

    // frame_id_ = std::string ("dvs0");
    // world_frame_id_ = std::string ("world");
    frame_id_ = "cam0";
    world_frame_id_ = "world";
    auto_trigger_ = false;

    // tf2::msg::TransformStamped inital_pose;
    // inital_pose.frame_id = world_frame_id_;
    // inital_pose.child_frame_id = "camera0";
    // inital_pose.timestamp = tf2::TimePointZero;
    // tf_.get()->setTransform(inital_pose, "tracker");

    c_ = PinholeCameraModel(shared_state_->m_calib_file_cam0);

    postCameraLoaded();
}

// Denna funktion ska kallas av tråden i main, konstruktorn kan inte kallas i en tråd
void Tracker::trackerRun(){
    std::thread pointcloud_thread(&Tracker::mapCallback, this);
    std::thread event_thread(&Tracker::eventCallback, this);
    // std::thread tf_thread(&Tracker::tfCallback, this);
    // // Setup Subscribers
    // event_sub_ = nh_.subscribe("events", 0, &Tracker::eventCallback, this);
    // map_sub_ = nh_.subscribe("pointcloud", 0, &Tracker::mapCallback, this);
    // remote_sub_ =
    //     nh_.subscribe("remote_key", 0, &Tracker::remoteCallback, this);
    // tf_sub_ = nh_.subscribe("tf", 0, &Tracker::tfCallback, this);
    // camera_info_sub_ =
    //     nh_.subscribe("camera_info", 1, &Tracker::cameraInfoCallback, this);

    // // Setup Publishers
    // poses_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("evo/pose", 0);
    // remote_pub_ = nh_.advertise<const std_msgs::String>("remote_key", 0);

    #ifdef TRACKER_DEBUG_REFERENCE_IMAGE
        std::thread map_overlap(&Tracker::publishMapOverlapThread, this);
        map_overlap.detach();
    #endif

    tracking_thread_ = std::thread(&Tracker::trackingThread, this);
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // initialize(tf2::TimePointZero);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // std::cout << keypoints_.size() << std::endl;

    while (true) {
        {
            std::lock_guard<std::mutex> lock(events_mutex_);
            if (events_.size() >= 100000) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cout << events_.size() << std::endl;
    if(first_event_){
        std::lock_guard<std::mutex> lock(events_mutex_);
        tf2::msg::TransformStamped inital_pose;
        inital_pose.frame_id = world_frame_id_;
        // inital_pose.child_frame_id = "camera0";
        inital_pose.child_frame_id = "cam0";
        // inital_pose.child_frame_id = frame_id_;
        int temp = events_.size() - 2*(events_.size() - 100000);
        inital_pose.timestamp = events_[temp].timestamp;
        // inital_pose.timestamp = events_.back().timestamp;
        // std::cout << tf2::timeToSec(events_.back().timestamp) << std::endl;
        {
            std::lock_guard<std::mutex> lock(shared_state_->pose_state_.pose_mtx);
            shared_state_->pose_state_.pose = inital_pose;
            shared_state_->pose_state_.event_stamp = temp;
            shared_state_->pose_state_.pose_ready = true;
            tf_.get()->setTransform(inital_pose, "tracker");
            shared_state_->pose_state_.pose_cv.notify_one();
        }
    }

    // std::cout << tf_.get()->allFramesAsYAML() << std::endl;

    event_thread.join();
    pointcloud_thread.join();
}
Tracker::~Tracker(){
    running_ = false;
    if (tracking_thread_.joinable()) {
        tracking_thread_.join();
    }
}

void Tracker::trackingThread() {
    const tf2::Duration interval = tf2::durationFromSec(0.01);
    tf2::TimePoint next_time = tf2::get_now() + interval;
    // LOG(INFO) << "Spawned tracking thread.";

    while (running_) {
        std::this_thread::sleep_until(next_time + tf2::durationFromSec(0.01));
        // std::cout << keypoints_.size() << std::endl;
        if (!idle_ && keypoints_.size() > 0) {
            std::cout << "running?" << std::endl;
            estimateTrajectory();
        }
    }
}

// TODO: Add this the msg to the shared_state. Is this needed even? How to handle this.
void Tracker::remoteCallback(const std::string& msg) {
    const std::string& cmd = msg;

    if (cmd == "switch")
        initialize(tf2::TimePointZero);
    else if (cmd == "reset")
    {
        // Reset the pose, re-start tracking and update the local map
        reset();
        std::string cmd_msg;
        cmd_msg = "switch";
        // remote_pub_.publish(cmd_msg);
        cmd_msg = "update";
        // remote_pub_.publish(cmd_msg);
    }
    else if (cmd == "bootstrap")
        auto_trigger_ = true;
}

// void Tracker::tfCallback() {
//     if (!idle_) return;

//     std::unique_lock<std::mutex> lock(shared_state_->pose_state_.pose_mtx);
//     while (true)
//     {
//         shared_state_->pose_state_.pose_cv.wait(lock, [this]{ return shared_state_->pose_state_.pose_ready; });
//         tf_.setTransform(shared_state_->pose_state_.pose, "tracker");
//         shared_state_->pose_state_.pose_ready = false;
//     }
//     // for (auto& msg : msgs) {
//     //     // TODO: Lägg till en autorhity som ett arguemnt i setTransform!
//     //     tf_.setTransform(msg, "tracker");
//     // }
// }

void Tracker::initialize(const tf2::TimePoint& ts) {
    // std::string bootstrap_frame_id = std::string("camera0");
    std::string bootstrap_frame_id = "cam0";
    // std::cout << "Target fram: " << bootstrap_frame_id << ", source frame: " << world_frame_id_ << std::endl;
    // std::string tf_debug;
    // tf_debug = tf_->allFramesAsString();
    // std::cout << tf_debug << std::endl;
    tf2::msg::TransformStamped TF_kf_world = tf_.get()->lookupTransform(bootstrap_frame_id, world_frame_id_, ts);
    // LOG(INFO) << "Completed lookup";
    Eigen::Affine3d T_kf_world(tf2::transformToEigen(TF_kf_world));

    T_world_kf_ = T_kf_world.cast<float>().inverse();
    T_kf_ref_ = Eigen::Affine3f::Identity();
    T_ref_cam_ = Eigen::Affine3f::Identity();

    while (cur_ev_ + 1 < events_.size() &&
           events_[cur_ev_].timestamp < TF_kf_world.timestamp)
        ++cur_ev_;
    // LOG(INFO) << "latest tf stamp: "<< tf2::timeToSec(TF_kf_world.timestamp);
    // LOG(INFO) << "latest event stamp: " << tf2::timeToSec(events_[cur_ev_].timestamp) << "id: " << cur_ev_ << " with events size:" << events_.size();
    // google::FlushLogFiles(google::INFO); // or WARNING, ERROR, etc.
    updateMap();
    idle_ = false;
}

void Tracker::reset() {
    idle_ = true;

    events_.clear();
    poses_.clear();
    poses_filtered_.clear();
    cur_ev_ = kf_ev_ = 0;
}

// Events are insted stored in the shared_state, these events are read and parsed
// in the mapper.
void Tracker::eventCallback() {
    static const bool discard_events_when_idle = false;
    std::unique_lock<std::mutex> lock(shared_state_->events_left_.mtx);
    while (true)
    {
        if (discard_events_when_idle && idle_) continue;
        clearEventQueue();

        // TODO: Gör det här bättre? Kolla på denna en gång till.
        shared_state_->events_left_.cv_event.wait(lock, [this]{ return shared_state_->events_left_.event_ready;});
        // events_ = shared_state_->events_left_.data;
        // for (const auto& event : shared_state_->events_left_.chunk) {
        //     std::cout << event.x << ", " << event.y << ", " << tf2::displayTimePoint(event.timestamp) << ", " << event.polarity << "\n";
        // }
        {
            std::lock_guard<std::mutex> lock(events_mutex_);
            for (const auto& e : shared_state_->events_left_.chunk) {
                events_.push_back(e);
            }
        }

        if(!first_event_){
            first_event_ = true;
        }
        shared_state_->events_left_.event_ready = false;
    }
}

void Tracker::mapCallback() {
    static size_t min_map_size = 0;
        // rpg_common_ros::param<int>(nhp_, "min_map_size", 0);

    std::unique_lock<std::mutex> lock(shared_state_->pcl_state_.pcl_mtx);
    while (true)
    {
        shared_state_->pcl_state_.pcl_cv.wait(lock, [this]{ return shared_state_->pcl_state_.pcl_ready;});

        /**
         * map_ and the pcl located in shared_state_ are almost the same.
         * Pcl in shared_state_ have points with a intesity (Point(XYZI)),
         * while map_ expectrs points of type Point(XZY).
         *
         * So just copy evetyhing over from pcl in shared_state_ to map_ except
         * the intensity
        */
        map_->header = shared_state_->pcl_state_.pcl.get()->header;
        map_->height = shared_state_->pcl_state_.pcl.get()->height;
        map_->width = shared_state_->pcl_state_.pcl.get()->width;
        map_->is_dense = shared_state_->pcl_state_.pcl.get()->is_dense == 1;
        // map_->resize(shared_state_->pcl_state_.pcl.get()->size());
        // FIX: Here a preformance gain is possible, pushback allocates memory
        // so could do it more staticlay
        map_->clear();
        for (const auto& pt : *shared_state_->pcl_state_.pcl.get()) {
            map_->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
        }

        // Old way, but should work. see explantation above why this is not used.
        // pcl::PCLPointCloud2 pcl_pc;
        // pcl::toPCLPointCloud2(*shared_state_->pcl_state_.pcl, pcl_pc);
        // pcl::fromPCLPointCloud2(pcl_pc, *map_);
        // LOG(INFO) << "Received new map: " << map_->size() << " points";

        if (map_->size() > min_map_size && auto_trigger_) {
            // LOG(INFO) << "Auto-triggering tracking";

            // initialize(msg->header.stamp);
            initialize(tf2::timeFromSec(shared_state_->pcl_state_.pcl.get()->header.stamp));
            auto_trigger_ = false;
        }

        shared_state_->pcl_state_.pcl_ready = false;
    }

}

void Tracker::updateMap() {
    static size_t min_map_size = 0;
    static size_t min_n_keypoints = 0;

    // std::cout << keypoints_.size() << std::endl;
    // std::cout << "map_ size: " << map_->size() << std::endl;
    if (map_->size() <= min_map_size) {
        // LOG(WARNING) << "Unreliable map! Can not update map.";
        return;
    }

    std::cout << keypoints_.size() << std::endl;
    T_kf_ref_ = T_kf_ref_ * T_ref_cam_;
    T_ref_cam_ = Eigen::Affine3f::Identity();
    kf_ev_ = cur_ev_;

    projectMap();
    std::cout << keypoints_.size() << std::endl;

    if (keypoints_.size() < min_n_keypoints) {
        // LOG(WARNING) << "Losing track!";
        // TODO: do something about it?
    }
}

void Tracker::clearEventQueue() {
    static size_t event_history_size_ = 5000000;
    // Batcha events här också?
    // TODO : Fixa så att tracker också läser events, inte bara mapper
    if (idle_) {
        // std::cout << "idle event cleaning" << std::endl;
        if (events_.size() > event_history_size_) {
            events_.erase(
                events_.begin(),
                events_.begin() + events_.size() - event_history_size_
            );

            cur_ev_ = kf_ev_ = 0;
        }
    } else {
        // std::cout << "event cleaning" << std::endl;
        events_.erase(events_.begin(), events_.begin() + kf_ev_);

        cur_ev_ -= kf_ev_;
        kf_ev_ = 0;
    }
}

void Tracker::publishMapOverlapThread() {
    static const float z0 = 1. / 10.,
                       z1 = 1. / .1, z_range = z1 - z0;
    const tf2::Duration interval = tf2::durationFromSec(0.01);
    tf2::TimePoint next_time = tf2::get_now() + interval;


    static cv::Mat cmap;
    if (!cmap.data) {
        cv::Mat gray(256, 1, CV_8U);
        for (int i = 0; i != gray.rows; ++i) gray.at<uchar>(i) = i;
        cv::applyColorMap(gray, cmap, cv::COLORMAP_JET);
    }

    // static image_transport::Publisher pub =
    //     it_.advertise("event_map_overlap", 1);

    cv::Mat ev_img, img;

    while (running_) {
        std::this_thread::sleep_until(next_time + tf2::durationFromSec(0.01));

        // pub.getNumSubscribers() == 0, kanske är bra att göra något liknande
        if (idle_ ||  event_rate_ < noise_rate_)
            continue;

        cv::convertScaleAbs(1. - .25 * new_img_, ev_img, 255);
        cv::cvtColor(ev_img, img, cv::COLOR_GRAY2RGB);

        Eigen::Affine3f T_cam_w =
            (T_world_kf_ * T_kf_ref_ * T_ref_cam_).inverse();

        const int s = 2;

        for (const auto& P : map_local_->points) {
            Eigen::Vector3f p = T_cam_w * Eigen::Vector3f(P.x, P.y, P.z);
            p[0] = p[0] / p[2] * fx_ + cx_;
            p[1] = p[1] / p[2] * fy_ + cy_;

            int x = std::round(s * p[0]), y = std::round(s * p[1]);
            float z = p[2];

            if (x < 0 || x >= s * width_ || y < 0 || y >= s * height_) continue;

            cv::Vec3b c = cmap.at<cv::Vec3b>(
                std::min(255., std::max(255. * (1. / z - z0) / z_range, 0.)));
            cv::circle(img, cv::Point(x, y), 2, cv::Scalar(c[0], c[1], c[2]),
                       -1, cv::LINE_AA, 1);
        }

        // TODO : Kolla på detta igen, vill vi publicera bilder. Kan ha en
        // #ifdef img_något för att aktivera detta? Liten optimisering
        // std_msgs::Header header;
        // header.stamp = events_[cur_ev_].ts;

        // sensor_msgs::ImagePtr msg =
        //     cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
        // pub.publish(msg);
    }
}

void Tracker::publishTF() {
    Eigen::Affine3f T_world_cam = T_world_kf_ * T_kf_ref_ * T_ref_cam_;

    tf2::msg::TransformStamped pose_tf;
    pose_tf = tf2::eigenToTransform(T_world_cam.cast<double>());
    pose_tf.timestamp = events_[cur_ev_ + frame_size_].timestamp;
    pose_tf.frame_id = world_frame_id_;
    // pose_tf.child_frame_id = "dvs_evo_raw";
    pose_tf.child_frame_id = "cam0";

    poses_.push_back(pose_tf);
    // {
    //     std::lock_guard<std::mutex> lock(shared_state_->pose_state_.pose_mtx);
    //     shared_state_->pose_state_.pose = pose_tf;
    //     std::cout << "new pose!" << std::endl;
    //     tf_.get()->setTransform(pose_tf, "tracker");
    //     shared_state_->pose_state_.pose_ready = true;
    //     shared_state_->pose_state_.pose_cv.notify_one();
    // }

    tf2::msg::TransformStamped filtered_pose;
    if (getFilteredPose(filtered_pose)) {
        filtered_pose.frame_id = world_frame_id_;
        filtered_pose.child_frame_id = frame_id_;
        // TODO: Göra något liknande för filtered_pose som för pose_tf
        poses_filtered_.push_back(filtered_pose);

        {
            std::lock_guard<std::mutex> lock(shared_state_->pose_state_.pose_mtx);
            shared_state_->pose_state_.pose = filtered_pose;
            std::cout << "new pose!" << std::endl;
            tf_.get()->setTransform(filtered_pose, "tracker");
            shared_state_->pose_state_.pose_ready = true;
            shared_state_->pose_state_.pose_cv.notify_one();
        }

        // publishPose();
    }
}

void Tracker::publishPose() {
    const tf2::msg::TransformStamped& T_world_cam = poses_.back();
    tf2::msg::TransformStamped pose;
    pose.timestamp = T_world_cam.timestamp;
    pose.frame_id = T_world_cam.frame_id;
    pose.transform.translation = T_world_cam.transform.translation;
    pose.transform.rotation = T_world_cam.transform.rotation;
    // TODO: Ska vi göra något här ockspå?
}

bool Tracker::getFilteredPose(tf2::msg::TransformStamped& pose) {
    static const size_t mean_filter_size = 10;

    if (mean_filter_size < 2) {
        pose = poses_.back();
        return true;
    }

    if (poses_.size() < mean_filter_size) {
        return false;
    }

    static Eigen::VectorXd P(7);
    P.setZero();

    // Take the first rotation q0 as the reference
    // Then, for the remainders rotations qi, instead of
    // averaging directly the qi's, average the incremental rotations
    // q0^-1 * q_i (in the Lie algebra), and then get the original mean
    // rotation by multiplying the mean incremental rotation on the left
    // by q0.

    Eigen::Quaterniond tf_q0 =
        poses_[poses_.size() - mean_filter_size].transform.rotation;
    const Quaternion q0(tf_q0.w(), tf_q0.x(), tf_q0.y(), tf_q0.z());
    const Quaternion q0_inv = q0.inverse();

    for (size_t i = poses_.size() - mean_filter_size; i != poses_.size(); ++i) {
        const Eigen::Quaterniond& tf_q = poses_[i].transform.rotation;
        const Quaternion q(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());
        const Quaternion q_inc = q0_inv * q;

        const Eigen::Vector3d& t = poses_[i].transform.translation;
        Transformation T(q_inc, t);

        P.head<6>() += T.log();
        P[6] += tf2::timeToSec(poses_[i].timestamp);
    }

    P /= mean_filter_size;
    Transformation T(Transformation::Vector6(P.head<6>()));

    const Eigen::Vector3d& t_mean = T.getPosition();
    const Quaternion q_mean = q0 * T.getRotation();

    // tf::StampedTransform filtered_pose;
    tf2::msg::TransformStamped filtered_pose;
    filtered_pose.transform.translation = t_mean;
    filtered_pose.transform.rotation = Eigen::Quaterniond(q_mean.w(), q_mean.x(), q_mean.y(), q_mean.z());
    // filtered_pose.setOrigin(tf::Vector3(t_mean[0], t_mean[1], t_mean[2]));
    // filtered_pose.setRotation(
        // tf::Quaternion(q_mean.x(), q_mean.y(), q_mean.z(), q_mean.w()));
    filtered_pose.timestamp = tf2::TimePoint(tf2::durationFromSec(P[6]));

    pose = filtered_pose;
    return true;
}

void Tracker::estimateTrajectory() {
    static const size_t max_event_rate = 8000000,
                        events_per_kf = 100000;
    // nhp_.param("max_event_rate", 8000000),
    // nhp_.param("events_per_kf", 100000);
    std::cout << "esimate" << std::endl;
    return;
    std::lock_guard<std::mutex> lock(data_mutex_);

    while (true) {
        if (cur_ev_ + std::max(step_size_, frame_size_) > events_.size()) break;

        if (cur_ev_ - kf_ev_ >= events_per_kf) updateMap();

        if (idle_) break;

        size_t frame_end = cur_ev_ + frame_size_;

        // Skip frame if event rate below noise rate
        double frameduration =
            (tf2::timeToSec(events_[frame_end].timestamp) - tf2::timeToSec(events_[cur_ev_].timestamp));
        event_rate_ =
            std::round(static_cast<double>(frame_size_) / frameduration);
        if (event_rate_ < noise_rate_) {
            // LOG(WARNING) << "Event rate below NOISE RATE. Skipping frame.";
            cur_ev_ += step_size_;
            continue;
        }
        if (event_rate_ > max_event_rate) {
            // LOG(WARNING) << "Event rate above MAX EVENT RATE. Skipping frame.";
            cur_ev_ += step_size_;
            continue;
        }
//        LOG(INFO) << "Event rate: "<< event_rate_;

        static size_t events_processed = 0, poses_generated = 0;
#ifdef TRACKING_PERF
        // Performance analysis
        static const size_t perf_interval = 2000000;
        static double time_elapsed = 0., last_ts = events_[cur_ev_].ts.toSec();

        if (events_processed > perf_interval) {
            double cur_ts = events_[cur_ev_].ts.toSec(),
                   time_processed = cur_ts - last_ts;

            LOG(INFO) << "   Poses/s: "
                      << static_cast<double>(poses_generated) / time_elapsed;
            LOG(INFO) << " Pose rate: "
                      << static_cast<double>(poses_generated) / time_processed;
            LOG(INFO) << "  Events/s: "
                      << static_cast<double>(events_processed) / time_elapsed;
            LOG(INFO) << "Event rate: "
                      << static_cast<double>(events_processed) / time_processed;
            LOG(WARNING) << " RT factor: " << time_processed / time_elapsed;

            events_processed = 0;
            poses_generated = 0;
            time_elapsed = 0;
            last_ts = cur_ts;
        }
        TIMER_START(t1);
#endif

        drawEvents(events_.begin() + cur_ev_, events_.begin() + frame_end,
                   new_img_);
#ifdef TRACKING_PERF
        {
            TIMER_STOP(t1, t2, duration);
            time_elapsed += duration;
            LOG(INFO) << "Forming event image required: " << duration << "ms";

            TIMER_START(t1);
        }
#endif
        cv::buildPyramid(new_img_, pyr_new_, pyramid_levels_);
        //LOG(INFO) << "*****************Collecting events from "<< events_[cur_ev_].ts << " - "<< events_[frame_end].ts;
        trackFrame();

        T_ref_cam_ *= SE3::exp(-x_).matrix();

        publishTF();        cur_ev_ += step_size_;

#ifdef TRACKING_PERF
        {
            TIMER_STOP(t1, t2, duration);
            time_elapsed += duration;
            LOG(INFO) << "Optimization required: " << duration << "ms";
        }
#endif

        events_processed += step_size_;
        ++poses_generated;
    }
}
