/*
* \file mapper_emvs_stereo.cpp
* \brief functions to create and process DSIs
* \author (1) Suman Ghosh
* \date 2024-09-29
* \author (2) Valentina Cavinato
* \date 2024-09-29
* \author (3) Guillermo Gallego
* \date 2024-09-29
* Copyright/Rights of Use:
* 2024, Technische Universität Berlin
* Prof. Guillermo Gallego
* Robotic Interactive Perception
* Marchstrasse 23, Sekr. MAR 5-5
* 10587 Berlin, Germany
*/

#include <mapper/mapper_emvs_stereo.hpp>
#include <mapper/median_filtering.hpp>
#include <pcl/filters/radius_outlier_removal.h>
// #include <mapper/mapper.hpp>

// #include <opencv2/photo.hpp> // inpaint
#include <opencv2/calib3d.hpp>

#include <fstream>

//#define TIMING_LOOP

namespace EMVS {

// using namespace geometry_utils;

MapperEMVS::MapperEMVS(const PinholeCameraModel& cam,
                       const ShapeDSI& dsi_shape)
    : dvs_cam_(cam)
    , dsi_shape_(dsi_shape)
{
    cv::Size full_resolution = cam.fullResolution();
    width_ = full_resolution.width;
    height_ = full_resolution.height;


//            LOG(INFO) << "dvs_cam_.K" << dvs_cam_.intrinsicMatrix();
//            LOG(INFO) << "dvs_cam_.D" << dvs_cam_.distortionCoeffs();
//            LOG(INFO) << "dvs_cam_.P" << dvs_cam_.fullProjectionMatrix();
//            LOG(INFO) << "dvs_cam_.R" << dvs_cam_.rotationMatrix();
//            LOG(INFO) << "dvs_cam_.fullResolution()" << dvs_cam_.fullResolution();

    // Set instrinsics of mapper using projection matrix of PinholeCameraModel cam
    K_ << dvs_cam_.fx(), 0.f, dvs_cam_.cx(),
            0.f, dvs_cam_.fy(), dvs_cam_.cy(),
            0.f, 0.f, 1.f;

//    for (int i = 0; i < 3; ++i) {
//            for (int j = 0; j < 3; ++j) {
//                K_(i, j) = static_cast<float>(dvs_cam_.intrinsicMatrix()(i, j));
//            }
//        }

    // LOG(INFO) << "K_ for this camera this: " << K_;

    setupDSI();

    // Set lens distortion model and precompute bearing vectors
    //  if (dvs_cam_.distortionCoeffs().total() == 4)
    //    distortion_model_ << "fisheye";
    //  else if (dvs_cam_.distortionCoeffs().total() == 5)
    //    distortion_model_ << "plumb_bob";
    //  else
    //  {
    //    LOG(ERROR) << "Unknown lens distortion model";
    //    exit(0);
    //  }'
    // Changed how the distortion model is set
    distortion_model_ << cam.distortionModel();
    // distortion_model_ << cam.camera_info().distortion_model;
    precomputeRectifiedPoints();
}

bool MapperEMVS::getPoseAt(std::shared_ptr<tf2::BufferCore> tf_, const tf2::TimePoint& t, std::string world_frame_id, std::string frame_id, Transformation& T) {
  std::string* error_msg = new std::string();
//   if(time == 0.653799){
//     std::cout << "Time: " << tf2::timeToSec(t) << std::endl;
//     std::cout << "World: " << world_frame_id << std::endl;
//     std::cout << "Frame id: " << frame_id << std::endl;
//   }

//   if(frame_id == "dvs1"){
//     std::cout << "Time: " << tf2::timeToSec(t) << std::endl;
//   }

//   if(time == 0.822208){
//     std::cout << "Time: " << tf2::timeToSec(t) << std::endl;
//   }

  if (!tf_->canTransform(world_frame_id, frame_id, t, error_msg)) {
      return false;
    } else {
        tf2::msg::TransformStamped tr = tf_->lookupTransform(world_frame_id, frame_id, t);
        // std::cout << frame_id << std::endl;
        // std::cout << "Time: " << tf2::timeToSec(t) << std::endl;

        // tf::transformTFToKindr(tr, &T);
        Transformation temp(tr.transform.rotation, tr.transform.translation);
        T = std::move(temp);
        return true;
    }
}

bool MapperEMVS::evaluateDSI(const std::vector<Event>& events,
                             std::shared_ptr<tf2::BufferCore> tf_,
                             const std::string world_frame_id,
                             const std::string cam_name,
                             const Transformation& T_rv_w)
{
    if(events.size() < packet_size_)
    {
        // LOG(WARNING) << "Number of events ( " << events.size() << ") < packet size (" << packet_size_ << ")";
        return false;
    }

    // 2D coordinates of the events transferred to reference view using plane Z = Z_0.
    // We use Vector4d because Eigen is optimized for matrix multiplications with inputs whose size is a multiple of 4
    static std::vector<Eigen::Vector4d> event_locations_z0;
    event_locations_z0.clear();

    // List of camera centers
    static std::vector<Eigen::Vector3d> camera_centers;
    camera_centers.clear();

    // DEBUGGING:
    std::cout << "[MapperEMVS::evaluateDSI] Beginning process of packaging events as frame_size_events..." << std::endl;

    // Loop through the events, grouping them in packets of frame_size_ events
    size_t current_event_ = 0;
    while(current_event_ + packet_size_ < events.size())
    {
        // DEBUGGING:
        //std::cout << "I HAVE ACCESSED WHILE LOOP" << std::endl;

        // Events in a packet are assigned the same timestamp (mid-point), for efficiency
        tf2::TimePoint frame_ts = events[current_event_ + packet_size_ / 2].timestamp;

        Transformation T_w_ev; // from event camera to world
        Transformation T_rv_ev; // from event camera to reference viewpoint

        if(cam_name == "dvs1"){
            std::cout << "Time for camera: "<< cam_name << ", " << tf2::timeToSec(frame_ts) << std::endl;
            std::cout << tf_->allFramesAsYAML(frame_ts) << std::endl;
        }
        // getPoseAt(tf_, frame_ts, world_frame_id, cam_name, T_w_ev) queries the transform buffer (tf_) to retrieve the
        // cameras pose (T_w_ev, from camera to world frame) at timestamp frame_ts for the camera named cam_name (e.g. cam0 or dvs1)

        // DEBUGGING:
        // std::cout << tf_->allFramesAsYAML() << std::endl;

        if(!getPoseAt(tf_, frame_ts, world_frame_id, cam_name, T_w_ev))
        {
            // std::cout << world_frame_id  << std::endl;
            // std::cout << cam_name << std::endl;
            // std::cout << tf2::timeToSec(frame_ts) << std::endl;
            current_event_++;
            continue;
        }

        // std::cout << "pose found" << std::endl;

        T_rv_ev = T_rv_w * T_w_ev;
        const Transformation T_ev_rv = T_rv_ev.inverse();
        // const Eigen::Matrix3d R = T_ev_rv.getRotationMatrix().cast<float>();
        // const Eigen::Vector3d t = T_ev_rv.getPosition().cast<float>();
        const Eigen::Matrix3d R = T_ev_rv.getRotationMatrix().cast<double>();
        const Eigen::Vector3d t = T_ev_rv.getPosition().cast<double>();

        // Optical center of the event camera in the coordinate frame of the reference view
        camera_centers.push_back(-R.transpose() * t);

        // Project the points on plane at distance z0
        const float z0 = raw_depths_vec_[0];

        // Planar homography  (H_z0)^-1 that maps a point in the reference view to the event camera through plane Z = Z0 (Eq. (8) in the IJCV paper)
        Eigen::Matrix3d H_z0_inv = R;
        H_z0_inv *= z0;
        H_z0_inv.col(2) += t;

        // Compute H_z0 in pixel coordinates using the intrinsic parameters
        Eigen::Matrix3d H_z0_inv_px = K_ * H_z0_inv * virtual_cam_.getKinv();

        // DEBUGGING:

        // if(cam_name == "dvs1"){
        //     std::cout << "FOR LEFT CAMERA" << std::endl;
        //     std::cout << "H_z0_inv:\n" << H_z0_inv << std::endl;
        //     std::cout << "K_:\n" << K_ << std::endl;
        //     std::cout << "Kinv:\n" << virtual_cam_.getKinv() << std::endl;
        //     std::cout << "T_w_ev:\n" << T_w_ev << std::endl;
        //     std::cout << "T_rv_w:\n" << T_rv_w << std::endl;
        //     std::cout << "T_ev_rv:\n" << T_ev_rv << std::endl;
        //     std::cout << "Translation t:\n" << t.transpose() << std::endl;
        //     std::cout << "T_ev_rv matrix for dvs1:\n" << T_ev_rv.getTransformationMatrix() << std::endl;
        //     std::cout << "Rotation:\n" << R << std::endl;
        //     //std::cout << "precomputed_rectified_points_:\n" << precomputed_rectified_points_ << std::endl;
        // }

        Eigen::Matrix3d H_z0_px = H_z0_inv_px.inverse();

        // Use a 4x4 matrix to allow Eigen to optimize the speed
        Eigen::Matrix4d H_z0_px_4x4;
        H_z0_px_4x4.block<3,3>(0,0) = H_z0_px;
        H_z0_px_4x4.col(3).setZero();
        H_z0_px_4x4.row(3).setZero();

        // For each packet, precompute the warped event locations according to Eq. (11) in the IJCV paper.
        for (size_t i=0; i < packet_size_; ++i)
        {
            const Event& e = events[current_event_++];
            Eigen::Vector4d p;
            p.head<2>() = precomputed_rectified_points_.col(e.y * width_ + e.x);

            // p.head<2>() = precomputed_rectified_points_.col(e.y * width_ + e.x).cast<double>();
            p[2] = 1.;
            p[3] = 0.;


            p = H_z0_px_4x4 * p;

            // if(cam_name == "cam0")
                // std::cout << "Before division p: " << p.transpose() << std::endl;

            p /= p[2];

            // if(cam_name == "dvs1"){
            //     std::cout << p << std::endl;
            //     std::cout << "WEIRD MATRIX:\n" << H_z0_px_4x4 << std::endl;
            //     // std::cout << "precomputed_rectified_points_:\n" << precomputed_rectified_points_ << std::endl;
            // }

            event_locations_z0.push_back(p);
        }
    }

    // DEBUGGING:
    std::cout << "[MapperEMVS::evaluateDSI] Events have successfully been packaged as frame_size_events." << std::endl;

    // if(cam_name == "dvs1"){
    //     // std::cout << p << std::endl;
    //     // std::cout << "WEIRD MATRIX:\n" << H_z0_px_4x4 << std::endl;
    //     std::cout << "precomputed_rectified_points_:\n" << precomputed_rectified_points_ << std::endl;
    // }

    dsi_.resetGrid();

    // DEBUGGING:
    //std::cout << "[MapperEMVS::evaluateDSI] DSI grid is successfully reset." << std::endl;
    // for(auto& event : event_locations_z0){
    //     std::cout << "EVENT: " << event << std::endl;
    // }
    fillVoxelGrid(event_locations_z0, camera_centers);

    // DEBUGGING:
    //std::cout << "[MapperEMVS::evaluateDSI] Voxel grid is sucessfully filled." << std::endl;

    //if(cam_name == "cam1")
    //    dsi_.printDataArray();

    return true;
}


void MapperEMVS::fillVoxelGrid(const std::vector<Eigen::Vector4d>& event_locations_z0,
                               const std::vector<Eigen::Vector3d>& camera_centers)
{
    // This function implements Step 2 of Algorithm 1 in the IJCV paper.
    // It maps events from plane Z0 to all the planes Zi of the DSI using Eq. (15)
    // and then votes for the corresponding voxel using bilinear voting.

    // DEBUGGING:
    //std::cout << "[MapperEMVS::fillVoxelGrid] Beginning process of back-projecting events into the DSI..." << std::endl;

    // For efficiency reasons, we split each packet into batches of N events each
    // which allows to better exploit the L1 cache
    static const int N = 128;
    typedef Eigen::Array<float, N, 1> Arrayf;

    const float z0 = raw_depths_vec_[0];

    // DEBUGGING:
    //std::cout << "[MapperEMVS::fillVoxelGrid] Setting up parallel threads..." << std::endl;

    // Parallelize over the planes of the DSI with OpenMP
    // (each thread will process a different depth plane)

#pragma omp parallel for num_threads(7) if (event_locations_z0.size() >= 20000)

    // DEBUGGING:
    //std::cout << "[MapperEMVS::fillVoxelGrid] Threads are up and running..." << std::endl;

    for(size_t depth_plane = 0; depth_plane < raw_depths_vec_.size(); ++depth_plane)
    {
        const Eigen::Vector4d* pe = &event_locations_z0[0];
        float *pgrid = dsi_.getPointerToSlice(depth_plane);

        for (size_t packet=0; packet < camera_centers.size(); ++packet)
        {
            // Precompute coefficients for Eq. (15)
            const Eigen::Vector3d& C = camera_centers[packet];
            const float zi = static_cast<float>(raw_depths_vec_[depth_plane]),
            // This is where the back-projection occurs
                    a = z0 * (zi - C[2]),
                    bx = (z0 - zi) * (C[0] * virtual_cam_.fx() + C[2] * virtual_cam_.cx()),
                    by = (z0 - zi) * (C[1] * virtual_cam_.fy() + C[2] * virtual_cam_.cy()),
                    d = zi * (z0 - C[2]);

            if(zi-C[2] < dsi_shape_.min_depth_){
                continue;
              }

            // Update voxel grid now, N events per iteration
            for(size_t batch=0; batch < packet_size_ / N; ++batch, pe += N)
            {
                // Eq. (15)
                Arrayf X, Y;
                for (size_t i=0; i < N; ++i)
                {
                    X[i] = pe[i][0];
                    Y[i] = pe[i][1];
                }
                X = (X * a + bx) / d;
                Y = (Y * a + by) / d;

                for (size_t i=0; i < N; ++i)
                {
                    // Bilinear voting
                    dsi_.accumulateGridValueAt(X[i], Y[i], pgrid);

                    // REMOVE THIS??!?!? Just causes issues with parallelization and seem to be for debugging only?!?
                    // Assuming you know the current depth_plane (k) for this slice:
                    //if(dvs_cam_.cam_name == "cam1"){
                    //    float updated_value = dsi_.getGridValueAt(X[i], Y[i], depth_plane);
                    //    if(updated_value != 0){
                    //        std::cout << "Updated grid value at (" << X[i] << "," << Y[i] << ") in slice "
                    //         << depth_plane << " is: " << updated_value << std::endl;
                    //     }
                    // }
                }
            }

            // DEBUGGING:
            //std::cout << "[MapperEMVS::fillVoxelGrid] Voxel Grid Successfully updated..." << std::endl;
        }
    }
}


void MapperEMVS::setupDSI()
{
    // CHECK_GT(dsi_shape_.min_depth_, 0.0);
    // CHECK_GT(dsi_shape_.max_depth_ , dsi_shape_.min_depth_);

    depths_vec_ = TypeDepthVector(dsi_shape_.min_depth_, dsi_shape_.max_depth_, dsi_shape_.dimZ_);
    raw_depths_vec_ = depths_vec_.getDepthVector();

    dsi_shape_.dimX_ = (dsi_shape_.dimX_ > 0) ? dsi_shape_.dimX_ : dvs_cam_.fullResolution().width;
    dsi_shape_.dimY_ = (dsi_shape_.dimY_ > 0) ? dsi_shape_.dimY_ : dvs_cam_.fullResolution().height;

    float f_virtual_cam_;
    if (dsi_shape_.fov_ < 10.f)
    {
        // LOG(INFO) << "Specified DSI FoV < 10 deg. Will use camera FoV instead.";
        f_virtual_cam_ = dvs_cam_.fx();
    }
    else
    {
        const float dsi_fov_rad = dsi_shape_.fov_ * CV_PI / 180.0;
        f_virtual_cam_ = 0.5 * (float) dsi_shape_.dimX_ / std::tan(0.5 * dsi_fov_rad);
    }
    // LOG(INFO) << "Focal length of virtual camera: " << f_virtual_cam_ << " pixels";

    //  virtual_cam_ = PinholeCamera(dsi_shape_.dimX_, dsi_shape_.dimY_,
    //                               f_virtual_cam_, f_virtual_cam_,
    //                               0.5 * (float)dsi_shape_.dimX_, 0.5 * (float)dsi_shape_.dimY_);


    virtual_cam_ = PinholeCameraModel(dsi_shape_.dimX_, dsi_shape_.dimY_,
                                 f_virtual_cam_, f_virtual_cam_,
                                 dvs_cam_.cx(), dvs_cam_.cy());
    dsi_ = Grid3D(dsi_shape_.dimX_, dsi_shape_.dimY_, dsi_shape_.dimZ_);
}


cv::Point2d fisheye_rectifyPoint(const cv::Point2d& uv_raw,
                                 const cv::Matx33d& K_, const cv::Mat& D_, const cv::Matx33d& R_, const cv::Matx34d& P_)
{
    // cv::Point2f raw32 = uv_raw, rect32;
    // const cv::Mat src_pt(1, 1, CV_32FC2, &raw32.x);
    // cv::Mat dst_pt(1, 1, CV_32FC2, &rect32.x);

    cv::Point2d raw64 = uv_raw, rect64;
    cv::Mat src_pt(1, 1, CV_64FC2, &raw64.x);
    cv::Mat dst_pt(1, 1, CV_64FC2, &rect64.x);

    cv::undistortPoints(src_pt, dst_pt, K_, D_, R_, P_);
    // cv::fisheye::undistortPoints(src_pt, dst_pt, K_, D_, R_, P_);
    return rect64;
}



void MapperEMVS::precomputeRectifiedPoints()
{
    // For fisheye distortion
    // För nuvarande är matriserna Eigen matriser, så måste bestäma vilket format
    cv::Matx33d K_ = dvs_cam_.intrinsicMatrix();
    cv::Mat D_ = dvs_cam_.distortionCoeffs();
    cv::Matx34d P_ = dvs_cam_.getProjectionMatrix();
    cv::Matx33d R_ = dvs_cam_.rotationMatrix();
    bool plumb_bob_distortion = !distortion_model_.str().compare("plumb_bob");
    bool fisheye_distortion = !distortion_model_.str().compare("fisheye");
    if (plumb_bob_distortion)
    {
        // LOG(INFO) << "plumb_bob_distortion = True";
    }
    if (fisheye_distortion)
    {
        // LOG(INFO) << "fisheye_distortion = True";
    }

    // Create a lookup table that maps pixel coordinates to undistorted pixel coordinates
    precomputed_rectified_points_ = Eigen::Matrix2Xd(2, height_ * width_);
    for(int y=0; y < height_; y++)
    {
        for(int x=0; x < width_; ++x)
        {
            cv::Point2d rectified_point;
            if (plumb_bob_distortion)
            {
                // Not using plumbed bob distortion
                // radial-tangential distortion
                // rectified_point = dvs_cam_.rectifyPoint(cv::Point2d(x,y));
            }
            else if (fisheye_distortion)
            {
                // fisheye distortion
                rectified_point = dvs_cam_.rectifyPoint(cv::Point2d(x,y));
                // rectified_point = fisheye_rectifyPoint(cv::Point2d(x,y), K_, D_, R_, P_);
            }
            else
            {
                // LOG(ERROR) << "Distortion model not set properly!";
                return;
            }
            // if(dvs_cam_.cam_name == "cam1")
            //     std::cout << "POINT: (" << rectified_point.x << "," << rectified_point.y << ")" << std::endl;
            precomputed_rectified_points_.col(y * width_ + x) = Eigen::Vector2d(rectified_point.x, rectified_point.y);
        }
    }

    // Center the rectified coordinates
    // Step 1: Compute the mean position of all rectified points
    Eigen::Vector2d mean_position = Eigen::Vector2d::Zero();
    for (int i = 0; i < height_ * width_; ++i)
    {
        mean_position += precomputed_rectified_points_.col(i);
    }
    mean_position /= (height_ * width_);

    // Step 2: Compute the translation to center the mean at (width/2, height/2)
    Eigen::Vector2d image_center(width_ / 2.0, height_ / 2.0);
    Eigen::Vector2d translation = image_center - mean_position;

    // Step 3: Apply the translation to all rectified points
    for (int i = 0; i < height_ * width_; ++i)
    {
        precomputed_rectified_points_.col(i) += translation;
    }

    // Step 4: Ensure all coordinates remain within image bounds (optional clamping)
    for (int i = 0; i < height_ * width_; ++i)
    {
        precomputed_rectified_points_.col(i).x() = std::max(0.0, std::min(precomputed_rectified_points_.col(i).x(), static_cast<double>(width_ - 1)));
        precomputed_rectified_points_.col(i).y() = std::max(0.0, std::min(precomputed_rectified_points_.col(i).y(), static_cast<double>(height_ - 1)));
    }

}


void MapperEMVS::convertDepthIndicesToValues(const cv::Mat &depth_cell_indices, cv::Mat &depth_map)
{
    // Convert depth indices to depth values, for all pixels
    depth_map = cv::Mat(depth_cell_indices.rows, depth_cell_indices.cols, CV_32F);
    // std::cout << depth_cell_indices.cols << std::endl;
    // std::cout << depth_cell_indices.rows << std::endl;
    // std::cout << depth_map.size() << std::endl;
    for(int y=0; y<depth_cell_indices.rows; ++y)
    {
        for(int x=0; x<depth_cell_indices.cols; ++x)
        {
            // std::cout << (int)depth_cell_indices.at<uchar>(y,x) << std::endl;
            depth_map.at<float>(y,x) = depths_vec_.cellIndexToDepth(depth_cell_indices.at<uchar>(y,x));
        }
    }
}


void MapperEMVS::removeMaskBoundary(cv::Mat& mask, int border_size)
{
    for(int y=0; y<mask.rows; ++y)
    {
        for(int x=0; x<mask.cols; ++x)
        {
            if(x <= border_size || x >= mask.cols - border_size ||
                    y <= border_size || y >= mask.rows - border_size)
            {
                mask.at<uchar>(y,x) = 0;
            }
        }
    }
}


void MapperEMVS::getDepthMapFromDSI(cv::Mat& depth_map, cv::Mat &confidence_map, cv::Mat &mask, const OptionsDepthMap &options_depth_map, int method)
{
    cv::Mat depth_map_dense;
    MapperEMVS::getDepthMapFromDSI(depth_map, confidence_map, mask, options_depth_map, depth_map_dense, method);
}


void MapperEMVS::getDepthMapFromDSI(cv::Mat& depth_map, cv::Mat &confidence_map, cv::Mat &semidense_mask, const OptionsDepthMap &options_depth_map, cv::Mat& depth_map_dense, int method)
{
    // Reference: Section 5.2.3 in the IJCV paper.

    // Maximum number of votes along optical ray
    cv::Mat depth_cell_indices;
    std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
#ifdef TIMING_LOOP
    int nloops = 100;
    for (int i=1; i<=nloops; i++){
#endif
        switch (method)
        {
        case 0:
            dsi_.collapseZSliceByLocalVar(&confidence_map, &depth_cell_indices);
            break;
        case 1:
            dsi_.collapseZSliceByLocalMeanSquare(&confidence_map, &depth_cell_indices);
            break;
        case 2:
            dsi_.collapseZSliceByGradMag(&confidence_map, &depth_cell_indices);
            break;
        case 3:
            dsi_.collapseZSliceByLaplacianMag(&confidence_map, &depth_cell_indices);
            break;
        case 4:
            dsi_.collapseZSliceByDoG(&confidence_map, &depth_cell_indices);
            break;
        default:
            dsi_.collapseMaxZSlice(&confidence_map, &depth_cell_indices);
            break;
        }

#ifdef TIMING_LOOP
    }
#endif
    std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
    auto t_max = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start ).count();
        // LOG(INFO) << "Time for argmax "<<t_max<<" ms";
    if (options_depth_map.full_sequence && options_depth_map.save_conf_stats){
        double min, max;
        cv::Mat conf_nonzero_mask = confidence_map>0;
        cv::minMaxLoc(confidence_map, &min, &max, NULL, NULL, conf_nonzero_mask);
        std::ofstream conf_range;
        std::string fname = "conf_range_" + name + ".txt";
        conf_range.open(fname, std::ofstream::app);
        conf_range << min << " " << max << std::endl;
        std::cout << min << " " << max << std::endl;
        conf_range.close();
    }

    // Adaptive thresholding on the confidence map
    cv::Mat confidence_8bit;
    // Fix confidence range before normalization to prevent pixels with few votes from appearing very confident during intervals with poor depth convergence
    confidence_map.at<float>(0, 0) = options_depth_map.max_confidence;
    cv::normalize(confidence_map, confidence_8bit, 0.0, 255.0, cv::NORM_MINMAX);

    confidence_8bit.at<float>(0, 0) = 0;
    confidence_8bit.convertTo(confidence_8bit, CV_8U);

    t_start = std::chrono::high_resolution_clock::now();
#ifdef TIMING_LOOP
    for (int i=1; i<=nloops; i++){
#endif


//std::cout << "THIS IS CONFIDENCE SO THAT FREDRIK CAN SEE: " << confidence_8bit << std::endl;
// std::cout << "THIS IS CONFIDENCE MAP: " << confidence_map << std::endl;


        cv::adaptiveThreshold(confidence_8bit,
                              semidense_mask,
                              1,
                              cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                              cv::THRESH_BINARY,
                              options_depth_map.adaptive_threshold_kernel_size_,
                              -options_depth_map.adaptive_threshold_c_);

#ifdef TIMING_LOOP
    }
#endif
    t_end = std::chrono::high_resolution_clock::now();
    auto t_agt = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start ).count()/1000.f;
        // LOG(INFO) << "Time for AGT "<<t_agt<<" ms";


    // Clean up depth map using median filter (Section 5.2.5 in the IJCV paper)
    cv::Mat depth_cell_indices_filtered;
    huangMedianFilter(depth_cell_indices,
                      depth_cell_indices_filtered,
                      semidense_mask,
                      options_depth_map.median_filter_size_);

    // Remove the outer border to suppress boundary effects
    const int border_size = std::max(options_depth_map.adaptive_threshold_kernel_size_ / 2, 1);
    removeMaskBoundary(semidense_mask, border_size);

    // Densify the depth map by inpainting
    cv::Mat inpaint_mask = 1 - semidense_mask;
    cv::Mat depth_cell_indices_inpainted;
    cv::inpaint(depth_cell_indices_filtered,inpaint_mask, depth_cell_indices_inpainted, 3, cv::INPAINT_TELEA);

    // Convert depth indices to depth values
    convertDepthIndicesToValues(depth_cell_indices_filtered, depth_map);
    convertDepthIndicesToValues(depth_cell_indices_inpainted, depth_map_dense);
}


void MapperEMVS::getPointcloud(
    const cv::Mat& depth_map,
    const cv::Mat& mask,
    const OptionsPointCloud &options_pc,
    PointCloud::Ptr &pc_
)
{
    // CHECK_EQ(depth_map.rows, mask.rows);
    // CHECK_EQ(depth_map.cols, mask.cols);

    // Convert depth map to point cloud
    pc_->clear();
    for(int y=0; y<depth_map.rows; ++y)
    {
        for(int x=0; x<depth_map.cols; ++x)
        {
            if(mask.at<uint8_t>(y,x) > 0)
            {
                BearingVector b_rv = virtual_cam_.projectPixelTo3dRay(Keypoint(x,y));
                b_rv.normalize();
                Eigen::Vector3d xyz_rv = (b_rv / b_rv[2] * depth_map.at<float>(y,x));

                pcl::PointXYZI p_rv; // 3D point in reference view
                p_rv.x = xyz_rv.x();
                p_rv.y = xyz_rv.y();
                p_rv.z = xyz_rv.z();
                p_rv.intensity = 1.0 / p_rv.z;
                pc_->push_back(p_rv);
            }
        }
    }

    // Filter point cloud to remove outliers (Section 5.2.5 in the IJCV paper)
    PointCloud::Ptr cloud_filtered (new PointCloud);
    pcl::RadiusOutlierRemoval<PointType> outlier_rm;
    outlier_rm.setInputCloud(pc_);
    outlier_rm.setRadiusSearch(options_pc.radius_search_);
    outlier_rm.setMinNeighborsInRadius(options_pc.min_num_neighbors_);
    outlier_rm.filter(*cloud_filtered);

    pc_->swap(*cloud_filtered);
}

void MapperEMVS::getPointcloud(
    const cv::Mat& depth_map,
    const cv::Mat& mask,
    const OptionsPointCloud &options_pc,
    PointCloud::Ptr &pc_,
    Transformation T_rv_w
)
{
    // CHECK_EQ(depth_map.rows, mask.rows);
    // CHECK_EQ(depth_map.cols, mask.cols);

    // Convert depth map to point cloud
    pc_->clear();
    Eigen::Matrix4d T_w_rv = T_rv_w.inverse().getTransformationMatrix().cast<double>();
    for(int y=0; y<depth_map.rows; ++y)
    {

        for(int x=0; x<depth_map.cols; ++x)
        {
            if(mask.at<uint8_t>(y,x) > 0)
            {
                BearingVector b_rv = virtual_cam_.projectPixelTo3dRay(Keypoint(x,y));
                b_rv.normalize();
                Eigen::Vector3d xyz_rv = (b_rv / b_rv[2] * depth_map.at<float>(y,x));
                Eigen::Vector3d xyz_world = T_w_rv.block<3,3>(0,0) * xyz_rv + T_w_rv.block<3,1>(0,3);

                pcl::PointXYZI p_w; // 3D point in reference view
                p_w.x = xyz_world.x();
                p_w.y = xyz_world.y();
                p_w.z = xyz_world.z();
                p_w.intensity = 1.0 / xyz_rv.z();
                // std::cout << "Point: ["
                // << p_w.x << ", "
                // << p_w.y << ", "
                // << p_w.z << "], Intensity: "
                // << p_w.intensity << std::endl;
                pc_->push_back(p_w);
            }
        }
    }

    // Filter point cloud to remove outliers (Section 5.2.5 in the IJCV paper)
    if (!pc_ || pc_->empty()) {
        std::cerr << "[ERROR] RadiusOutlierRemoval: input cloud is null or empty." << std::endl;
        return;
    }
    PointCloud::Ptr cloud_filtered (new PointCloud);
    pcl::RadiusOutlierRemoval<PointType> outlier_rm;
    outlier_rm.setInputCloud(pc_);
    outlier_rm.setRadiusSearch(options_pc.radius_search_);
    outlier_rm.setMinNeighborsInRadius(options_pc.min_num_neighbors_);
    outlier_rm.filter(*cloud_filtered);

    pc_->swap(*cloud_filtered);
}

}
