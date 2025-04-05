#ifndef SHARED_STATE_HPP
#define SHARED_STATE_HPP

#include <mutex>
#include <condition_variable>
#include <string>
#include <queue>

#include <tf2/msg/pose_stamped.hpp>
#include <tf2/time.hpp>
#include "tf2/buffer_core.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Other states that should be stored, not sure currently


struct tf_state {
    std::mutex tf_mtx;
    std::condition_variable tf_cv;
    bool tf_ready = false;
    std::shared_ptr<tf2::BufferCore> tf_;
};

struct pcl_state {
    std::mutex pcl_mtx;
    std::condition_variable pcl_cv;
    bool pcl_ready = false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl;
};

struct pose_state {
    std::mutex pose_mtx;
    // Behövs condition variable?
    std::condition_variable pose_cv;
    bool pose_ready = false;
    tf2::msg::PoseStamped pose;
};

inline void waitForTransform(
    std::string &target_frame,
    std::string &source_frame,
    tf2::TimePoint &time,
    tf2::Duration &timeout
    // ,tf_state &tf_state
)
{
    // tf_state.tf_;
}

class SharedState{
private:

public:
    SharedState();
    ~SharedState() = default;

    pcl_state pcl_state_;
    pose_state pose_state_;
    tf_state tf_state_;
    std::mutex data_mutex;

    inline void waitForTransform(
        std::string &target_frame,
        std::string &source_frame,
        tf2::TimePoint &time,
        // ,tf_state &tf_state
        tf2::Duration &timeout)
    {
        // tf_state_.tf_.get()->addTransformableRequest;
    }

};

#endif