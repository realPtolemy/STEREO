#include "mapper/mapper.hpp"

Mapper::Mapper(SharedState &shared_state) :
	shared_state_(&shared_state),
	tf_(shared_state_->tf_)
{
	cam0 = PinholeCameraModel(shared_state_->m_calib_file_cam0);
	cam1 = PinholeCameraModel(shared_state_->m_calib_file_cam1);
	cam0.readStereoCalib(shared_state_->stereo_calibfile_, mat4_1_0);
	cam0.cam_name = "cam0";
	cam1.cam_name = "cam1";
	mat4_hand_eye = Eigen::Matrix4d::Identity();
	// mat4_hand_eye = cam0.getHandEye().inverse() * cam1.getHandEye();
	// mat4_1_0 = mat4_hand_eye;
	// std::cout << mat4_1_0 << std::endl;

	EMVS::ShapeDSI dsi_shape(dimX, dimY, dimZ, fov_deg, min_depth, max_depth);
	initialized_ = false;
	latest_tf_stamp_ = tf2::TimePointZero;
	current_ts_ = tf2::TimePointZero;
	state_= IDLE;
	auto_trigger_ = true;
	pc_ = EMVS::PointCloud::Ptr(new EMVS::PointCloud);
	pc_global_ = EMVS::PointCloud::Ptr(new EMVS::PointCloud);

	min_duration_ = .1;
	max_duration_ = 1.;
	init_wait_t_ = 0;
	// init_wait_t_ = 3.;

	world_frame_id_ = "world";
	frame_id_ = "cam0";
	// regular_frame_id_ = "dvs0";
	// bootstrap_frame_id_ = "camera0";
	// bootstrap_frame_id_ = regular_frame_id_;
	// frame_id_ = bootstrap_frame_id_;
	// frame_id_ = regular_frame_id_;

    cv::Size full_resolution = cam0.fullResolution();
    event_image0_ = cv::Mat(full_resolution,CV_8UC1);
    event_image1_ = cv::Mat(full_resolution,CV_8UC1);
    // tf_ = std::make_shared<tf2::BufferCore>(tf2::durationFromSec(100) );
    tf_->setUsingDedicatedThread(true);
}

void Mapper::mapperRun(){
	std::vector<Event> camera1_events, camera2_events;
	camera1_events.reserve(EVENT_BATCH_SIZE);
	camera2_events.reserve(EVENT_BATCH_SIZE);

	#ifdef READ_EVENT_FROM_AESTREAM
		UDP camera1_server(4001);
		UDP camera2_server(4002);
		std::thread camera1_thread(&Mapper::camera_thread_udp, this, std::ref(camera1_server), std::ref(camera1_events), std::ref(shared_state_->events_left_));
		std::thread camera2_thread(&Mapper::camera_thread_udp, this, std::ref(camera2_server), std::ref(camera2_events), std::ref(shared_state_->events_right_));
		// 3party/aestream_src/bin/aestream input file data/camera_0.csv output udp
	#else
	// google::FlushLogFiles(google::INFO);
		std::thread camera1_thread_csv(
			&Mapper::camera_thread_csv,
			this,
			"data/camera_0.csv",
			std::ref(camera1_events),
			std::ref(shared_state_->events_left_)
		);
		std::thread camera2_thread_csv(
			&Mapper::camera_thread_csv,
			this,
			"data/camera_1.csv",
			std::ref(camera2_events),
			std::ref(shared_state_->events_right_)
		);
	#endif
	std::thread pose_reader(&Mapper::tfCallback, this);

	mapper_thread_ = std::thread(&Mapper::mappingLoop, this);

	#ifdef READ_EVENT_FROM_AESTREAM
		camera1_thread.join();
		camera2_thread.join();
	#else
	camera1_thread_csv.join();
	camera2_thread_csv.join();
	#endif

	pose_reader.join();
}

Mapper::~Mapper()
{
	running_ = false;
    if (mapper_thread_.joinable()) {
        mapper_thread_.join();
    }
}

void Mapper::tfCallback(){
	std::unique_lock<std::mutex> lock(shared_state_->pose_state_.pose_mtx);
	while (true)
	{
		shared_state_->pose_state_.pose_cv.wait(lock, [this]{ return shared_state_->pose_state_.pose_ready; });
		std::chrono::high_resolution_clock::time_point t_start_callback = std::chrono::high_resolution_clock::now();
		// tf_->setTransform(shared_state_->pose_state_.pose, "mapper");
		if(shared_state_->pose_state_.pose.child_frame_id == "hand"){
            tf2::TimePoint tf_stamp_= shared_state_->pose_state_.pose.timestamp;
			tf2::msg::TransformStamped T_hand_eye = tf2::eigenToTransform(Eigen::Affine3d(mat4_hand_eye));
			T_hand_eye.timestamp = tf_stamp_;
			T_hand_eye.frame_id = "hand";
			T_hand_eye.child_frame_id = frame_id_;
			// T_hand_eye.child_frame_id = bootstrap_frame_id_;
            // Broadcast hand eye transform
			// TODO: update pose evetying that listens to tf here, that is pose.
            // broadcaster_.sendTransform(stamped_T_hand_eye);
		}
		if (shared_state_->pose_state_.pose.child_frame_id == frame_id_ || shared_state_->pose_state_.pose.child_frame_id == "/"+frame_id_) {
			std::cout << "Give second camerea a pose!" << std::endl;
			tf2::TimePoint tf_stamp_= shared_state_->pose_state_.pose.timestamp;
			tf2::msg::TransformStamped T_0_1, T_0_2;
			T_0_1 = tf2::eigenToTransform(Eigen::Affine3d(mat4_1_0));
			int temp = shared_state_->pose_state_.event_stamp;
			T_0_1.timestamp = shared_state_->events_left_.data[temp].timestamp;
			// T_0_1.timestamp = tf_stamp_;
			// std::cout << tf2::timeToSec(T_0_1.timestamp) << std::endl;
			T_0_1.frame_id = frame_id_;
			T_0_1.child_frame_id = "dvs1";
			// std::cout << tf2::timeToSec(tf_stamp_) << std::endl;
            tf_->setTransform(T_0_1, "mapper");
			std::cout << tf_->allFramesAsYAML() << std::endl;

			// tf_->lookupTransform("cam0", "dvs1", T_0_1.timestamp);


            // if (frame_id_ == bootstrap_frame_id_) {
            //     // keep bootstrap frame also as regular frame for future use
            //     shared_state_->pose_state_.pose.child_frame_id = regular_frame_id_;
            //     tf_->setTransform(shared_state_->pose_state_.pose, "mapper");
            // }
		}
		std::chrono::high_resolution_clock::time_point t_end_callback = std::chrono::high_resolution_clock::now();
		shared_state_->pose_state_.pose_ready = false;
	}
}

bool Mapper::waitForTransformSimple(
	const std::shared_ptr<tf2::BufferCore> & buffer,
	const std::string & target_frame,
	const std::string & source_frame,
	const tf2::TimePoint & time,
	const tf2::Duration & timeout,
	const tf2::Duration & polling_sleep)
{
	auto start = std::chrono::steady_clock::now();
	if (buffer->canTransform(target_frame, source_frame, time)) {
		return true;
	}
	else{
		return false;
	}
}

void Mapper::mappingLoop()
{
    dsi_shape = EMVS::ShapeDSI(dimX, dimY, dimZ, min_depth, max_depth, fov_deg);

	opts_depth_map.max_confidence = max_confidence;
	opts_depth_map.adaptive_threshold_kernel_size_ = adaptive_threshold_kernel_size;
	opts_depth_map.adaptive_threshold_c_ = adaptive_threshold_c;
	opts_depth_map.median_filter_size_ = median_filter_size;
	opts_depth_map.full_sequence = full_seq;
	opts_depth_map.save_conf_stats = save_conf_stats;
	opts_depth_map.save_mono = save_mono;
	opts_depth_map.rv_pos = rv_pos;

	bool map_initialized = false;

    const tf2::Duration interval = tf2::durationFromSec(0.01);
    tf2::TimePoint next_time = tf2::get_now() + interval;
	std::this_thread::sleep_until(next_time + tf2::durationFromSec(0.01));

	std::string error_msg;
	// std::cout << tf2::timeToSec(current_ts_) << std::endl;
	while (running_){
		if (!shared_state_->pcl_state_.pcl_listeners){
			pc_global_->clear();
		}

        tf2::msg::TransformStamped latest_tf;

		if(waitForTransformSimple(
			tf_,
			world_frame_id_,
			frame_id_,
			tf2::TimePointZero,
			tf2::Duration(0),
			tf2::durationFromSec(0.01))
		){
			latest_tf = tf_->lookupTransform(world_frame_id_, frame_id_, tf2::TimePointZero);
			// std::cout << "lookup succeded!" << std::endl;
			latest_tf_stamp_ = latest_tf.timestamp;

		} else {
			// LOG(WARNING) << error_msg;
			continue;
		}
        if (auto_trigger_ && initialized_ && !map_initialized && tf2::durationToSec(latest_tf_stamp_ - current_ts_) > init_wait_t_) {
			// LOG(INFO) << "GENERATING INITIAL MAP AUTOMATICALLY.";
            state_ = MAPPING;
			std::cout << "Mapping" << std::endl;
        }

        if (state_ != MAPPING){
			// TODO: Handle this?
			std::cout << "TIMING" << std::endl;
			continue;
		}
		std::vector<Event> ev_subset_left_, ev_subset_right_, ev_subset_tri_;
		int last_tracked_ev_left, last_tracked_ev_right;
		if (current_ts_ >= latest_tf_stamp_) {
			continue;
		}
		{
			std::lock_guard<std::mutex> lock(data_mutex_);
			// Find the first events that's older than the previous saved tf.
			last_tracked_ev_left = shared_state_->events_left_.data.size() -1;
			while (last_tracked_ev_left>0 && shared_state_->events_left_.data[last_tracked_ev_left].timestamp > latest_tf_stamp_){
				--last_tracked_ev_left;
			}

			// The same for the right camera
			last_tracked_ev_right = shared_state_->events_right_.data.size() -1;
			while (last_tracked_ev_right>0 && shared_state_->events_right_.data[last_tracked_ev_right].timestamp > latest_tf_stamp_){
				--last_tracked_ev_right;
			}

			// Check that there is enough events in both cameras to make a new map
			if (last_tracked_ev_left <= NUM_EV_PER_MAP || last_tracked_ev_right <= NUM_EV_PER_MAP){
				// std::cout << "Left: "<< last_tracked_ev_left << std::endl;
				// std::cout << "Right: "<< last_tracked_ev_left << std::endl;
				continue;
			}

			current_ts_ = latest_tf_stamp_;
			double_t duration = tf2::durationToSec(current_ts_ - shared_state_->events_left_.data[last_tracked_ev_left - NUM_EV_PER_MAP].timestamp);
			if (duration > max_duration_){
				// LOG(INFO) << "Looking too far back in the past. skip";
				std::cout << "Looking too far back in the past. skip" << std::endl;
				continue;
			}
			if (duration < min_duration_){
				// LOG(INFO) << "Time interval is not big enough. There might be flashing events. Skip.";
				std::cout << "Time interval is not big enough. There might be flashing events. Skip." << std::endl;

				continue;
			}
			ev_subset_left_ = std::vector<Event>(
				shared_state_->events_left_.data.begin()+last_tracked_ev_left-NUM_EV_PER_MAP,
				shared_state_->events_left_.data.begin()+last_tracked_ev_left
			);
			ev_subset_right_ = std::vector<Event>(
				shared_state_->events_right_.data.begin()+last_tracked_ev_right-NUM_EV_PER_MAP,
				shared_state_->events_right_.data.begin()+last_tracked_ev_right
			);
		}
		std::cout <<"------------------- Mapping at time: " << tf2::timeToSec(current_ts_)-tf2::timeToSec(first_ev_ts) << " seconds ----------------------" << std::endl;
		std::cout <<"------------------- Mapping at time: " << tf2::timeToSec(current_ts_) << " ---------------------" << std::endl;
		std::cout <<"------------------- Pose from: " << frame_id_ << " ---------------------" << std::endl;

		std::cout <<"Using left events from " << tf2::timeToSec(ev_subset_left_.front().timestamp)-tf2::timeToSec(first_ev_ts) << "-" << tf2::timeToSec(ev_subset_left_.back().timestamp) - tf2::timeToSec(first_ev_ts) << std::endl;
		std::cout <<"Using left events from " << tf2::timeToSec(ev_subset_left_.front().timestamp) << "-" << tf2::timeToSec(ev_subset_left_.back().timestamp) << std::endl;

		std::cout <<"Using right events from " << tf2::timeToSec(ev_subset_right_.front().timestamp) - tf2::timeToSec(first_ev_ts) << "-" << tf2::timeToSec(ev_subset_right_.back().timestamp) - tf2::timeToSec(first_ev_ts) << std::endl;
		std::cout <<"Using right events from " << tf2::timeToSec(ev_subset_right_.front().timestamp) << "-" << tf2::timeToSec(ev_subset_right_.back().timestamp) << std::endl;
		MappingAtTime(current_ts_, ev_subset_right_, ev_subset_left_, ev_subset_tri_, frame_id_);

		if (on_demand_)
			state_ = IDLE;
		if (auto_copilot_ && !map_initialized)
			frame_id_ = regular_frame_id_;
		map_initialized = true;
	}
}

void Mapper::MappingAtTime(
	tf2::TimePoint current_ts,
	std::vector<Event> &events_left_,
	std::vector<Event> &events_right_,
	std::vector<Event> &events_tri_,
	std::string frame_id
){
    // LOG(INFO)<<"Initializing mapper fused";
    EMVS::MapperEMVS mapper_fused(cam0, dsi_shape); //, mapper_fused_camera_time(cam0, dsi_shape);
    mapper_fused.name = "fused";
    // LOG(INFO)<<"Initializing mapper 0";
    EMVS::MapperEMVS mapper0(cam0, dsi_shape);
    mapper0.name="0";
    // LOG(INFO)<<"Initializing mapper 1";
    EMVS::MapperEMVS mapper1(cam1, dsi_shape);
    mapper1.name="1";

    // LOG(INFO)<<"Initializing mapper 2";
    EMVS::MapperEMVS mapper2(cam2, dsi_shape);
    mapper2.name="2";


	std::cout << "choosing method" << std::endl;
    switch(process_method)
    {
    case 1:
        // Compute two DSIs (one for each camera) and fuse them.
        // Only fusion across stereo camera, i.e., Alg. 1 of MC-EMVS (Ghosh and Gallego, AISY 2022) is implemented here
        process_1(
			world_frame_id_,
			frame_id,
			cam0,
			cam1,
			cam2,
			tf_,
			events_left_,
			events_right_,
			events_tri_,
			opts_depth_map,
			dsi_shape,
			mapper_fused,
			mapper0,
			mapper1,
			mapper2,
			out_path,
			current_ts,
			stereo_fusion,
			T_rv_w_
		);
        break;
    default:
        // LOG(INFO) << "Incorrect process method selected.. exiting";
        exit(0);
    }
	std::cout << "method chosen" << std::endl;

    // Convert DSI to depth maps using argmax and noise filtering
    mapper_fused.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map);
    //    mapper0.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map);

    // Convert semi-dense depth map to point cloud
    EMVS::OptionsPointCloud opts_pc;
    opts_pc.radius_search_ = radius_search;
    opts_pc.min_num_neighbors_ = min_num_neighbors;
    mapper_fused.getPointcloud(depth_map, semidense_mask, opts_pc, pc_, T_rv_w_);
    //    mapper0.getPointcloud(depth_map, semidense_mask, opts_pc, pc_, T_rv_w_);



	publishMsgs(frame_id);
}

void Mapper::publishMsgs(std::string frame_id){
	#ifdef PUBLISH_IMG
		publishImgs(frame_id);
	#endif
	// TODO: Add a bool flag here, representing that the tracker is activated and wants pcl
	if(!pc_->empty()){
		pc_->header.stamp = tf2::timeToSec(current_ts_);
		std::lock_guard<std::mutex> lock(shared_state_->pcl_state_.pcl_mtx);
		shared_state_->pcl_state_.pcl = pc_;
		shared_state_->pcl_state_.pcl_ready = true;
		shared_state_->pcl_state_.pcl_cv.notify_all();
		// TODO: en global pointcloud? Den ackumulerar flera pointclouds
	}
}

void Mapper::publishImgs(std::string frame_id){
	// LOG(INFO) << "Publishing images!!!!!!!!!!!!!!!!!!!!!!!!! ";
    // cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    // cv_ptr->encoding = "mono8";
    // cv_ptr->header.stamp = ros::Time(current_ts_);
    // cv_ptr->header.frame_id = frame_id;
    // cv_ptr->image = event_image0_;
    // ev_img_pub_.publish(cv_ptr->toImageMsg());
	std::cout << "publish images with id: " << frame_id << std::endl;
	// cv::imwrite("images/event_image0_.png", event_image0_);

    // if (invDepthMap_pub_.getNumSubscribers() > 0 || conf_map_pub_.getNumSubscribers() > 0) {

    //     // Save confidence map as an 8-bit image
    //     cv::Mat confidence_map_255;
    //     cv::normalize(confidence_map, confidence_map_255, 0, 255.0, cv::NORM_MINMAX, CV_8UC1);

    //     // colorize depth maps according to max and min depth
    //     cv::Mat invdepthmap_8bit, invdepthmap_color;
    //     cv::Mat invmap = 1./depth_map;
    //     float mod_max_depth = 1 * dsi_shape.max_depth_;
    //     cv::Mat invdepth_map_255 = (invmap - 1./mod_max_depth)  / (1./dsi_shape.min_depth_ - 1./mod_max_depth) * 255.;
    //     invdepth_map_255.convertTo(invdepthmap_8bit, CV_8U);
    //     cv::applyColorMap(invdepthmap_8bit, invdepthmap_color, cv::COLORMAP_JET);
    //     cv::Mat invdepth_on_canvas = cv::Mat(depth_map.rows, depth_map.cols, CV_8UC3, cv::Scalar(1,1,1)*0);
    //     invdepthmap_color.copyTo(invdepth_on_canvas, semidense_mask);
    //     cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(3,3));
    //     cv::dilate(invdepth_on_canvas, invdepth_on_canvas, element);

    //     // Change the background from black to white
    //     for ( int i = 0; i < invdepth_on_canvas.rows; i++ ) {
    //         for ( int j = 0; j < invdepth_on_canvas.cols; j++ ) {
    //             if ( invdepth_on_canvas.at<cv::Vec3b>(i, j) == cv::Vec3b(0, 0, 0) ) {
    //                 invdepth_on_canvas.at<cv::Vec3b>(i, j)[0] = 255;
    //                 invdepth_on_canvas.at<cv::Vec3b>(i, j)[1] = 255;
    //                 invdepth_on_canvas.at<cv::Vec3b>(i, j)[2] = 255;
    //             }
    //         }
    //     }

    //     //      saveDepthMaps(depth_map, confidence_map, semidense_mask, dsi_shape.min_depth_, dsi_shape.max_depth_, std::string("fused"), FLAGS_out_path + std::to_string(ts));

    //     cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    //     cv_ptr->encoding = "bgr8";
    //     cv_ptr->header.stamp = ros::Time(current_ts_);
    //     cv_ptr->header.frame_id = "/map";
    //     cv_ptr->image = invdepth_on_canvas;
    //     invDepthMap_pub_.publish(cv_ptr->toImageMsg());

    //     cv_bridge::CvImagePtr cv_ptr2(new cv_bridge::CvImage);
    //     cv_ptr2->encoding = "mono8";
    //     cv_ptr2->header.stamp = ros::Time(current_ts_);
    //     cv_ptr2->header.frame_id = "/map";
    //     cv_ptr2->image = 255 - confidence_map_255;
    //     conf_map_pub_.publish(cv_ptr2->toImageMsg());
    // }
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
void Mapper::camera_thread_csv(const std::string &event_file_path, std::vector<Event> &camera_events, EventQueue<Event> &event_queue)
{
  	std::ifstream event_file(event_file_path);
	if(!event_file){
		std::cerr << "Unable to open file: "<< event_file_path << std::endl;
		return;
	}
  	std::string line;
  	while (std::getline(event_file, line)) {
		// std::cout << "Size of a line: "<< line.size() << std::endl;
		Event e = parse_line(line);
  		camera_events.push_back(e);
  		if (camera_events.size() == EVENT_BATCH_SIZE){

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

/**
 * Reads events from a csv file. A batch of events are read in at first.
 * One batch contains many events, but the last event (last line) is most likely partially complete
 * So save that leftover from previous batch and add it to the beginin of new batch, and then
 * the event is whole and correct!
 */
void Mapper::camera_thread_csv_new(const std::string &event_file_path, EventQueueGrouped<SyncedEvents> &event_queue)
{
	std::ifstream file(event_file_path);
    if (!file) {
		std::cerr << "Error opening file.\n";
        return;
    }
	const size_t BUFFER_SIZE = 1 << 16;
    std::vector<char> buffer(BUFFER_SIZE);
    std::string leftover;
	SyncedEvents grouped_events;

    while (file) {
        file.read(buffer.data(), buffer.size());
        std::streamsize bytesRead = file.gcount();
        if (bytesRead <= 0) break;

		// std::cout << "leftover: " << leftover << std::endl;

        // Combine leftover + current buffer
        std::string data = leftover + std::string(buffer.data(), bytesRead);

        size_t pos = 0;
        while (true) {
            size_t newline = data.find('\n', pos);
            if (newline == std::string::npos) break;

            std::string lines = data.substr(pos, newline - pos);
            pos = newline + 1;

			std::this_thread::sleep_for(std::chrono::microseconds(100));

			parse_lines(lines, event_queue, grouped_events);
        }

        // Save leftover (incomplete last line)
        leftover = data.substr(pos);
    }

	// Add the last grouped events.
	if (!grouped_events.events.empty()) {
		event_queue.push_back(grouped_events);
	}


    // If the last line doesn't end with '\n'
    if (!leftover.empty()) {
        std::cout << "Parsed last (incomplete) line: " << leftover << "\n";
    }

	file.close();

}
void Mapper::parse_lines(const std::string& lines, EventQueueGrouped<SyncedEvents> &event_queue, SyncedEvents &grouped_events)
{
	std::istringstream ss(lines);
	std::string line;

	tf2::TimePoint timestamp;

	std::string token;
	std::istringstream lineStream;
	std::vector<std::string> tokens;
	while(std::getline(ss, line)){
		lineStream.clear();
		lineStream.str(line);
		tokens.clear();
		while (std::getline(lineStream, token, ',')) {
			tokens.push_back(token);
		}

		if (tokens.size() < 4) continue;

		Event e;
		uint64_t ts_us = std::stoull(tokens[0]);
		timestamp = tf2::TimePoint(tf2::durationFromSec(ts_us * 1e-6));

		e.x = std::stoi(tokens[1]);
		e.y = std::stoi(tokens[2]);
		e.polarity = std::stoi(tokens[3]);

		if(timestamp != grouped_events.timestamp){
			{
				std::lock_guard<std::mutex> lock(data_mutex_);
				if(!initialized_){
					first_ev_ts = timestamp;
					initialized_ = true;
					current_ts_ = first_ev_ts;
				}
			}
			if (grouped_events.events.empty()) {
				// First time: initialize the group
				std::cout << "first event!" << std::endl;
				grouped_events.timestamp = timestamp;
				grouped_events.events.push_back(e);
				continue;
			}
			// Timestamp changed: finalize previous and start new
			event_queue.push_back(grouped_events);
			grouped_events = SyncedEvents();
			grouped_events.timestamp = timestamp;
		}
		grouped_events.events.push_back(e);
	}
}
void Mapper::camera_thread_udp(UDP& udp, std::vector<Event> &camera_events, EventQueue<Event> &event_queue)
{

	while (true) {
		const uint16_t* data;
		ssize_t count;
		std::tie(data, count) = udp.receive_aestream();
		if(count == 0){
			continue;;
		}
		parse_bufferd_data(data, count, camera_events);
		std::lock_guard<std::mutex> lock(data_mutex_);
		if (!initialized_) {
			first_ev_ts = camera_events[0].timestamp;
			initialized_ = true;
			current_ts_ = first_ev_ts;
		}
		event_queue.push_back(std::move(camera_events));
		camera_events.clear();
	}
}


void Mapper::parse_bufferd_data(const uint16_t* buffered_data, ssize_t length, std::vector<Event> &camera_events){
	/**
	* 	The event data that aestream sent is in the format:
	* 	First 32 bits:
	* 			Upper 16 bits is x coordinate
	* 			Upper 16 bits MSB signals the packet has a timestamp
	* 			Lower 16 bits is y coordinate
	* 			Lower 16 bits MSB is the polarity
	* 	Reamning 32 bits is the timestamp
	*/
	for(int i = 0; i < length; i+=4){
		Event e;
		e.x = buffered_data[i] & 0x7FFF;
		e.y = buffered_data[i+1] & 0x7FFF;
		e.polarity = buffered_data[1+1] & 0x8000;
		uint32_t ts_us = (static_cast<uint32_t>(buffered_data[i + 3]) << 16) | buffered_data[i + 2];
		e.timestamp = tf2::TimePoint(tf2::durationFromSec(ts_us * 1e-6));
		camera_events.push_back(std::move(e));
	}
}

Event Mapper::parse_line(const std::string& line)
{
	std::istringstream ss(line);
	std::string token;
	std::vector<std::string> tokens;
	while (std::getline(ss, token, ',')) {
		tokens.push_back(token);
	}
	Event e;
	uint64_t ts_us = std::stoull(tokens[0]);
	e.timestamp = tf2::TimePoint(tf2::durationFromSec(ts_us * 1e-6));

	e.x = std::stoi(tokens[1]);
	e.y = std::stoi(tokens[2]);
	e.polarity = std::stoi(tokens[3]);
	return e;
}