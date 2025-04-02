#ifndef SHARED_STATE_HPP
#define SHARED_STATE_HPP

#include <mutex>
#include <condition_variable>
#include <string>

#include <tf2/msg/pose_stamped.hpp>
#include <tf2/time.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Other states that should be stored, not sure currently

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



#endif