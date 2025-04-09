#ifndef SHARED_STATE_HPP
#define SHARED_STATE_HPP

#include <mutex>
#include <condition_variable>
#include <string>
#include <queue>

#include "event.hpp"
#include "udp/udp.hpp"

#include <tf2/msg/pose_stamped.hpp>
#include <tf2/time.hpp>
#include "tf2/buffer_core.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Other states that should be stored, not sure currently

template <typename T>
struct EventQueue {
    std::mutex mtx;
    std::deque<T> data;

	void push_back(T&& chunk) {
		std::lock_guard<std::mutex> lock(mtx);
		data.push_back(std::move(chunk));
		checkSize();
	}

private:
    // Copied from ES-PTAM, added so that it takes a queue of vectors of events instead
	void checkSize() {
		static const size_t MAX_EVENT_QUEUE_LENGTH = 50000000 / 512;
		if (data.size() > MAX_EVENT_QUEUE_LENGTH) {
			size_t to_remove = data.size() - MAX_EVENT_QUEUE_LENGTH;
			std::cout << "Event queue is too long, removing " << to_remove << " events\n";
			data.erase(data.begin(), data.begin() + to_remove);
		}
	}
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

class SharedState{
private:

public:
    SharedState() {};
    ~SharedState() = default;

    pcl_state pcl_state_;
    pose_state pose_state_;
    EventQueue<std::vector<Event>> events_left_;
    EventQueue<std::vector<Event>> events_right_;
    std::mutex data_mutex;

    // inline void waitForTransform(
    //     std::string &target_frame,
    //     std::string &source_frame,
    //     tf2::TimePoint &time,
    //     // ,tf_state &tf_state
    //     tf2::Duration &timeout)
    // {
    //     // tf_state_.tf_.get()->addTransformableRequest;
    // }

};

#endif