#include "mapper/mapper.hpp"

Mapper::Mapper(SharedState &shared_state)
{
	shared_state_ = &shared_state;
	PinholeCameraModel cam0(m_calib_file_cam1, camera1), cam1(m_calib_file_cam2, camera2);

	EMVS::ShapeDSI dsi_shape(dimX, dimY, dimZ, fov_deg, min_depth, max_depth);
	initialized_ = false;
	latest_tf_stamp_ = tf2::TimePointZero;
	current_ts_ = tf2::TimePointZero;

	std::vector<Event> camera1_events, camera2_events;
	camera1_events.reserve(NUM_EV_PER_MAP);
	camera2_events.reserve(NUM_EV_PER_MAP);
	std::vector<std::thread> threads(NUM_OF_CAMERAS_);

	pc_ = EMVS::PointCloud::Ptr(new EMVS::PointCloud);

	#ifdef READ_EVENT_FROM_AESTREAM
	/**
	 * First a connection to the UDP stream has to be made.
	 */
	Server camera1_server(3333);
	// Server camera2_server(port2)
	std::thread camera1_thread(&Mapper::camera_thread_udp, this, camera1_server, std::ref(camera1_events), std::ref(shared_state_->events_left_));
	// 3party/aestream_src/bin/aestream input file data/camera_0.csv output udp
	// std::thread camera2_thread(camera_thread_udp, camera2_server, std::ref(camera2_events), std::ref(shared_state_->events_right_));
	#else
	threads[0] = std::thread(&Mapper::camera_thread_csv, this, "data/camera_0.csv", std::ref(camera1_events), std::ref(shared_state_->events_left_));
	threads[1] = std::thread(&Mapper::camera_thread_csv, this, "data/camera_1.csv", std::ref(camera2_events), std::ref(shared_state_->events_right_));
	#endif
	std::thread mapper_thread(&Mapper::mappingLoop, this);
	// std::cout << "DSI merger" << std::endl;

	// dsi_merger(std::ref(camera1_events), std::ref(camera2_events));
	#ifdef READ_EVENT_FROM_AESTREAM
		camera1_thread.join();
	#else
		threads[0].join();
		threads[1].join();
	#endif

	mapper_thread.join();
}

void Mapper::mappingLoop()
{
	while (true){
		if(false){
			pc_global_->clear();
		}

        tf2::msg::TransformStamped latest_tf;
        // if(tf_->waitForTransform(world_frame_id_, frame_id_, tf2::TimePointZero, ros::Duration(0), ros::Duration(0.01), &error_msg)){
        //     tf_->lookupTransform(world_frame_id_, frame_id_, tf2::TimePointZero, latest_tf);
        //     latest_tf_stamp_ = latest_tf.stamp_;
        // }
		// else {
		// 	// LOG(WARNING) << error_msg;
		// 	continue;
		// }

		// std::vector<Event> ev_subset_left_, ev_subset_right_;
		// int last_tracked_ev_left, last_tracked_ev_right;
		// if (current_ts_ < latest_tf_stamp_)
	}

}

void dsi_merger(std::vector<Event> &camera1_events, std::vector<Event> &camera2_events)
{
    std::cout << "DSI merger" << std::endl;
}

/*
	Camera thread reads in a stream of event data and batches them into a vector of events.
	When the vector is full, it is moved into a event queue.
	This ensures that the vector of events are not copied, which can be expensive.
	Also, when moving the events into the queue, the vector of events is empty afterwards.

	This code is not direclty from ES-PTAM, more inspried (mainly by the use of a queue).
	The main difference here is that, events are batched instead and also that this is not a callback function

	Good idead to have some params for docuemntation here.
*/
void Mapper::camera_thread_csv(const std::string &event_file_path, std::vector<Event> &camera_events, EventQueue<std::vector<Event>> &event_queue)
{
  	// ha en if sats eller #ifdef för att kolla ifall det är udp eller csv
  	std::ifstream event_file(event_file_path);
	if(!event_file){
		std::cerr << "Unable to open file: "<< event_file_path << std::endl;
		return;
	}
  	std::string line;
  	while (std::getline(event_file, line)) {
		Event e = parse_line(line);
  		camera_events.push_back(e);
  		if (camera_events.size() == NUM_EV_PER_MAP){
			std::lock_guard<std::mutex> lock(data_mutex_);
			if (!initialized_) {
				// Behvös en event_offset?
				first_ev_ts = camera_events[0].timestamp;
				initialized_ = true;
				current_ts_ = first_ev_ts;
			}
			event_queue.push_back(std::move(camera_events));
			// Move should empty the vector, camera_events.size() should be 0
			// but this is safer.
			camera_events.clear();
		}
	}
  	event_file.close();
}

void Mapper::camera_thread_udp(Server server, std::vector<Event> &camera_events, EventQueue<std::vector<Event>> &event_queue){

	while (true) {
		// std::string buffered_data = server.receive();
		// server.receive();
		server.receive_aestream();
		/**
		 * 	The event data that aestream sent is in the format:
		 * 	First 32 bits:
		 * 			Upper 16 bits is x coordinate
		 * 			Upper 16 bits MSB signals the packet has a timestamp
		 * 			Lower 16 bits is y coordinate
		 * 			Lower 16 bits MSB is the polarity
		 * 	Reamning 32 bits is the timestamp
		 */
		// if (buffered_data.empty()) {
		// 	// A packet can be lost on the way, the stream should not ebd becuse of that.
		// 	continue;
		// }

		// parse_bufferd_data(buffered_data);
		// std::cout << buffered_data << "\n";
	}
}

// Copied from ES-PTAM, added so that it takes a queue of vectors of events instead
// void Mapper::checkEventQueue(std::deque<std::vector<Event>> &event_queue)
// {
// 	// 50000000
//     static const size_t MAX_EVENT_QUEUE_LENGTH = 50000000/NUM_EV_PER_MAP;
//     if (event_queue.size() > MAX_EVENT_QUEUE_LENGTH)
//     {
// 		std::cout << "Event queue is too long, removing " << event_queue.size() << " events" << std::endl;
//         size_t NUM_EVENTS_TO_REMOVE = event_queue.size() - MAX_EVENT_QUEUE_LENGTH;
//         event_queue.erase(event_queue.begin(), event_queue.begin() + NUM_EVENTS_TO_REMOVE);
//     }
// }

/*
	Parse a line from a csv file and return an Event object.
	Could be good to implemnt the time defintion from tf2, so convert the uint64_t to a timestamp,
	what tf uses.
*/
// Event Mapper::parse_bufferd_data(std::string &buffered_data){
// 	/**
// 	 * A event is 8 bytes, with the timestamp
// 	 */
// 	int current_start_pos = 0;
// 	int current_end_pos = 8;
// 	std::string event_data;
// 	while (current_end_pos != buffered_data.length())
// 	{
// 		event_data = buffered_data.substr(current_start_pos, current_end_pos);

// 	}

// 	// std::stringstream ss(buffered_data);
// }

Event Mapper::parse_line(const std::string& line)
{
	std::istringstream ss(line);
	std::string token;
	std::vector<std::string> tokens;
	while (std::getline(ss, token, ',')) {
		tokens.push_back(token);
	}
	Event e;
	e.timestamp = tf2::TimePoint(tf2::Duration(std::stoull(tokens[0])));
	e.x = std::stoi(tokens[1]);
	e.y = std::stoi(tokens[2]);
	e.polarity = std::stoi(tokens[3]);
	return e;
}