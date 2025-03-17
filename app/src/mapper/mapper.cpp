#include "mapper/mapper.hpp"

Mapper::Mapper()
{
	CameraInfo camera1, camera2;

	PinholeCameraModel camera1_model(m_calib_file_cam1, camera1),
			camer2_model(m_calib_file_cam2, camera2);

	Eigen::Matrix4d mat4_1_0, mat4_2_0, mat4_hand_eye;

	EMVS::ShapeDSI dsi_shape(dimX, dimY, dimZ, for_deg, min_depth, max_depth);

	std::vector<event> camera1_events, camera2_events;
	camera1_events.reserve(NUM_EVENTS);
	camera2_events.reserve(NUM_EVENTS);
	// std::array<std::thread, 2> threads;
	std::vector<std::thread> threads(m_NUM_OF_CAMERAS);

	// std::vector<std::vector<event>> camera_events;
	// camera_events.push_back(camera1_events);
	// camera_events.push_back(camera2_events);
	// Med koden ovan kan man göra en loop som går igenom den yttersta arrayen och skapar trådar för varje innerarray
	// Detta kräver dock att programmet vet i förväg var filerna ligger någonstans
	// Sedan kommer detta inte alltid göras med csv filer, utan även med UDP paket

	// std::thread camera1_thread(local_dsi_csv, "data/camera_0.csv", std::ref(camera1_events));
	// std::thread camera2_thread(local_dsi_csv, "data/camera_1.csv", std::ref(camera2_events));
	threads[0] = std::thread(&Mapper::local_dsi_csv, this, "data/camera_0.csv", std::ref(camera1_events), 0);
	threads[1] = std::thread(&Mapper::local_dsi_csv, this, "data/camera_1.csv", std::ref(camera2_events), 1);

	// while (true)
	// {
	// 	std::unique_lock<std::mutex> lock(m_merge_dsi_mutex);
	// 	m_merge_dsi_cv.wait(lock, [this]{ return m_merge_ready; });

	// 	// DSI Merging
	// 	// std::cout << "DSI merger" << std::endl;
	// 	// dsi_merger(camera1_events, camera2_events);

	// 	// Reset merge_ready flag
	// 	m_merge_ready = false;
	// 	lock.unlock();
	// }


	// dsi_merger(std::ref(camera1_events), std::ref(camera2_events));
	threads[0].join();
	threads[1].join();
}

void dsi_merger(std::vector<event> &camera1_events, std::vector<event> &camera2_events)
{
    std::cout << "DSI merger" << std::endl;
}

void Mapper::local_dsi_csv(const std::string &event_file_path, std::vector<event> &camera_events, int id)
{
  	// ha en if sats eller #ifdef för att kolla ifall det är udp eller csv
  	std::ifstream event_file(event_file_path);
	if(!event_file){
		std::cerr << "Unable to open file: "<< event_file_path << std::endl;
	}
  	std::string line;
  	while (std::getline(event_file, line)) {
  		event e = parse_line(line);
  		camera_events.push_back(e);
  		if (camera_events.size() != NUM_EVENTS){
			continue;
  		}

		// JAG HATAR SKIT MULTITHREADING, AVLSUTA MITT LIDANDE!
		// JAG ÄR FÖRVIRRAD, JAG VET INTE VAD JAG GÖR

  		// Här ska en "batch" av events göras till en DSI
  		// Sedan ska de två "lokala" DSIerna mergas till en "global" DSI
  		// Synkorinsera DSIerna innan de mergas
  		// std::cout << "Number of events reached" << std::endl;
  		// Data processing, compute DSI
		// std::unique_lock<std::mutex> lck(m_local_dsi_mutex);
  		// m_threads++;
  		// if (m_threads < m_NUM_OF_CAMERAS){
		// 	m_local_dsi_cv.wait(lck, [this]{ return m_threads == 0; });
		// 	std::cout << "Thread " << id << " is done" << std::endl;
		// } else {
		// 	m_threads = 0;
  		// 	m_local_dsi_cv.notify_all();
		// 	std::cout << "Thread " << id << " reset the counter\n" << std::endl;
		// }
		// lck.unlock();
		// // {
		// 	std::lock_guard<std::mutex> lck(m_merge_dsi_mutex);
		// 	m_merge_ready = true;
		// }
		// m_merge_dsi_cv.notify_one();
		camera_events.clear();
		std::cout << event_file_path << std::endl;
		// break;
	}
  	event_file.close();
}

event Mapper::parse_line(const std::string& line)
{
	std::istringstream ss(line);
	std::string token;
	std::vector<std::string> tokens;
	while (std::getline(ss, token, ',')) {
		tokens.push_back(token);
	}
	event e;
	e.x = std::stoi(tokens[0]);
	e.y = std::stoi(tokens[1]);
	e.timestamp = std::stoi(tokens[2]);
	e.polarity = std::stoi(tokens[3]);
	return e;
}