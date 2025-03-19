#include <cartesian3dgrid/cartesian3dgrid.h>
// #include <pcl/point_types.h>
/*
Struct ShapeDSI is taken from https://github.com/tub-rip/dvs_mcemvs
*/
namespace EMVS
{
struct ShapeDSI
{
public:
    ShapeDSI(){}

    ShapeDSI(size_t dimX, size_t dimY, size_t dimZ, float min_depth, float max_depth, float fov)
        : dimX_(dimX)
        , dimY_(dimY)
        , dimZ_(dimZ)
        , min_depth_(min_depth)
        , max_depth_(max_depth)
        , fov_(fov) {}

    size_t dimX_;
    size_t dimY_;
    size_t dimZ_;

    float min_depth_;
    float max_depth_;

      // Field of View
    float fov_;
};
}

struct OptionsDepthMap
{
  // Adaptive Gaussian Thresholding parameters
  int adaptive_threshold_kernel_size_;
  double adaptive_threshold_c_;
  double max_confidence;
  bool full_sequence;
  bool save_conf_stats;
  bool save_mono;
  double rv_pos;
  // Kernel size of median filter
  int median_filter_size_;
};


struct OptionsPointCloud
{
  // Outlier removal parameters
  float radius_search_;
  int min_num_neighbors_;
};

// Denna del är modifierad
// typedef LinearTrajectory TrajectoryType;

// class MapperEMVS
// {
// public:

//   MapperEMVS(){}
//   MapperEMVS& operator=(const MapperEMVS& m){return *this;}

//   MapperEMVS(const image_geometry::PinholeCameraModel& cam,
//              const ShapeDSI &dsi_shape);

//   bool evaluateDSI(const std::vector<dvs_msgs::Event>& events,
//                    const TrajectoryType& trajectory,
//                    const geometry_utils::Transformation& T_rv_w);

//   void getDepthMapFromDSI(cv::Mat& depth_map, cv::Mat &confidence_map, cv::Mat &mask, const OptionsDepthMap &options_depth_map, int method=-1);
//   void getDepthMapFromDSI(cv::Mat& depth_map, cv::Mat &confidence_map, cv::Mat &mask, const OptionsDepthMap &options_depth_map, cv::Mat& depth_map_dense, int method=-1);

//   void getPointcloud(const cv::Mat& depth_map,
//                             const cv::Mat& mask,
//                             const OptionsPointCloud &options_pc,
//                             PointCloud::Ptr &pc_);

//   Grid3D dsi_;
//   std::string name;


// private:

//   void setupDSI();

//   void precomputeRectifiedPoints();

//   void fillVoxelGrid(const std::vector<Eigen::Vector4f> &event_locations_z0,
//                      const std::vector<Eigen::Vector3f> &camera_centers);

//   void convertDepthIndicesToValues(const cv::Mat &depth_cell_indices, cv::Mat &depth_map);

//   void removeMaskBoundary(cv::Mat& mask, int border_size);


//   // Intrinsics of the camera
//   image_geometry::PinholeCameraModel dvs_cam_;
//   Eigen::Matrix3f K_;
//   int width_;
//   int height_;
//   std::stringstream distortion_model_;

//   // (Constant) parameters that define the DSI (size and intrinsics)
//   ShapeDSI dsi_shape_;
//   geometry_utils::PinholeCamera virtual_cam_;

//   // Precomputed vector of num_depth_cells_ inverse depths,
//   // uniformly sampled in inverse depth space
//   TypeDepthVector depths_vec_;
//   std::vector<float> raw_depths_vec_;

//   // Precomputed (normalized) bearing vectors for each pixel of the reference image
//   Eigen::Matrix2Xf precomputed_rectified_points_;

//   const size_t packet_size_ = 1024;

// };

// }
