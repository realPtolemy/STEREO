#ifndef SHARED_STATE_HPP
#define SHARED_STATE_HPP

#include <mutex>
#include <condition_variable>
#include <string>
#include <queue>

#include "app/event.hpp"
#include "udp/udp.hpp"

// #include <glog/logging.h>
#include <tf2/msg/pose_stamped.hpp>
#include <tf2/time.hpp>
#include "tf2/buffer_core.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Other states that should be stored, not sure currently

typedef pcl::PointCloud<pcl::PointXYZI>::Ptr Pointcloud;

template <typename T>
struct EventQueue
{
    std::mutex mtx;
    std::condition_variable cv_event;
    bool event_ready = false;
    std::deque<T> data;
    std::vector<Event> chunk;

	void push_back(std::vector<Event>&& chunk) {
		std::lock_guard<std::mutex> lock(mtx);
        this->chunk = chunk;
        for(Event &e : chunk){
            data.push_back(std::move(e));
        }
		// data.push_back(std::move(chunk));
		checkSize();
        event_ready = true;
        cv_event.notify_one();
	}
private:
    // Copied from ES-PTAM, added so that it handles a queue of vectors of events instead
	void checkSize() {
		static const size_t MAX_EVENT_QUEUE_LENGTH = 50000000;
		if (data.size() > MAX_EVENT_QUEUE_LENGTH) {
			size_t to_remove = data.size() - MAX_EVENT_QUEUE_LENGTH;
			std::cout << "Event queue is too long, removing " << to_remove << " events\n";
			data.erase(data.begin(), data.begin() + to_remove);
		}
	}
};

struct pcl_state
{
    std::mutex pcl_mtx;
    std::condition_variable pcl_cv;
    bool pcl_ready = false;
    bool pcl_listeners = false;
    Pointcloud pcl;
};

struct pose_state
{
    std::mutex pose_mtx;
    // Behövs condition variable?
    std::condition_variable pose_cv;
    bool pose_ready = false;
    int event_stamp;
    tf2::msg::TransformStamped pose;
};

class SharedState
{
private:

public:
    SharedState() {
        tf_ = std::make_shared<tf2::BufferCore>();
    };
    ~SharedState() = default;

    const std::string m_calib_file_cam0 =
        "calibration/calibrationData/MONO_camera_1_left_2025MAY01.yaml";
    const std::string m_calib_file_cam1 =
        "calibration/calibrationData/MONO_camera_0_right_2025MAY01.yaml";
    const std::string stereo_calibfile_ =
        "calibration/calibrationData/STEREO_both_cameras_2025MAY01.yaml";

    std::shared_ptr<tf2::BufferCore> tf_ = std::make_shared<tf2::BufferCore>(tf2::durationFromSec(5));
    pcl_state pcl_state_;
    pose_state pose_state_;
    EventQueue<Event> events_left_;
    EventQueue<Event> events_right_;
    std::mutex data_mutex;
};

#endif