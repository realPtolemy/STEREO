#include "mapper/camera.hpp"
#include "event.hpp"
#include "mapper/mapper_emvs_stereo.hpp"

#include <iostream>
#include <vector>
#include <fstream>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp> // For image display
#include <opencv2/imgproc.hpp> // For image processing

#define dimX 0
#define dimY 0
#define dimZ 100
#define for_deg 0.0
#define min_depth 0.3
#define max_depth 5.0
#define NUM_EVENTS 512

const std::string calib_file_cam1 =
    "calibration/calibrationData/calibration_data_camera_0.yaml";
const std::string calib_file_cam2 =
    "calibration/calibrationData/calibration_data_camera_1.yaml";

event parse_line(const std::string& line);
void local_dsi(const std::string &event_file_path, std::vector<event> &camera1_events);

static std::condition_variable cv;
static std::mutex mtx;

void mapper(){
    CameraInfo camera1, camera2;

    PinholeCameraModel camera1_model(calib_file_cam1, camera1),
        camer2_model(calib_file_cam2, camera2);

    Eigen::Matrix4d mat4_1_0, mat4_2_0, mat4_hand_eye;

    EMVS::ShapeDSI dsi_shape(dimX, dimY, dimZ, for_deg, min_depth, max_depth);

    std::vector<event> camera1_events, camera2_events;
    camera1_events.reserve(NUM_EVENTS);
    camera2_events.reserve(NUM_EVENTS);
    std::thread camera1_thread(local_dsi, "data/camera_0.csv", std::ref(camera1_events));
    std::thread camera2_thread(local_dsi, "data/camera_1.csv", std::ref(camera2_events));
    camera1_thread.join();
    camera2_thread.join();
}

void local_dsi(const std::string &event_file_path, std::vector<event> &camera_events)
{
    // ha en if sats eller #ifdef för att kolla ifall det är udp eller csv
    std::ifstream event_file(event_file_path);
    if (event_file.is_open()){
      std::string line;
      while (std::getline(event_file, line)) {
        event e = parse_line(line);
        camera_events.push_back(e);
        if (camera_events.size() == NUM_EVENTS){
            // Här ska en "batch" av events göras till en DSI
            // Sedan ska de två "lokala" DSIerna mergas till en "global" DSI
            // Synkorinsera DSIerna innan de mergas
            std::cout << "Number of events reached" << std::endl;


            camera_events.clear();
            break;
        }
      }
      event_file.close();
    } else {
      std::cerr << "Unable to open file" << std::endl;
    }
}

event parse_line(const std::string& line)
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