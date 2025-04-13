#ifndef MAPPER_EMVS_STEREO_HPP
#define MAPPER_EMVS_STEREO_HPP

#include "camera.hpp"
#include "event.hpp"
#include "mapper/depth_vector.hpp"
#include "mapper/transformation.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "cartesian3dgrid/cartesian3dgrid.h"
#include "tf2/time.hpp"
#include "tf2/buffer_core.hpp"
/*
Struct ShapeDSI is taken from https://github.com/tub-rip/dvs_mcemvs
Though some things are changed, such as differnt event, pinhole camera imports.
*/
namespace EMVS
{
  typedef pcl::PointXYZI PointType;
  typedef pcl::PointCloud<PointType> PointCloud;

  #ifdef USE_INVERSE_DEPTH
    using TypeDepthVector = InverseDepthVector;
  #else
    using TypeDepthVector = LinearDepthVector;
  #endif

  struct ShapeDSI
  {
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

  class MapperEMVS
  {
  public:

    MapperEMVS(){}
    MapperEMVS& operator=(const MapperEMVS& m){return *this;}

    MapperEMVS(
      const PinholeCameraModel& cam,
      const ShapeDSI &dsi_shape
    );

    bool evaluateDSI(const std::vector<Event>& events,
      std::shared_ptr<tf2::BufferCore> tf_,
      const std::string world_frame_id,
      const std::string cam_name,
      const Transformation& T_rv_w);

    void getDepthMapFromDSI(
      cv::Mat& depth_map,
      cv::Mat &confidence_map,
      cv::Mat &mask,
      const OptionsDepthMap &options_depth_map,
      int method=-1
  );
    void getDepthMapFromDSI(
      cv::Mat& depth_map,
      cv::Mat &confidence_map,
      cv::Mat &mask,
      const OptionsDepthMap &options_depth_map,
      cv::Mat& depth_map_dense,
      int method=-1
    );

    void getPointcloud(
      const cv::Mat& depth_map,
      const cv::Mat& mask,
      const OptionsPointCloud &options_pc,
      PointCloud::Ptr &pc_
    );

    void getPointcloud(
      const cv::Mat& depth_map,
      const cv::Mat& mask,
      const OptionsPointCloud &options_pc,
      PointCloud::Ptr &pc_,
      Transformation T_rv_w
    );

    bool getPoseAt(
      std::shared_ptr<tf2::BufferCore> tf_,
      const tf2::TimePoint& t,
      std::string world_frame_id,
      std::string frame_id,
      Transformation& T
    );

    Grid3D dsi_;
    std::string name;


  private:

    void setupDSI();

    void precomputeRectifiedPoints();

    void fillVoxelGrid(
      const std::vector<Eigen::Vector4d> &event_locations_z0,
      const std::vector<Eigen::Vector3d> &camera_centers
    );

    void convertDepthIndicesToValues(
      const cv::Mat &depth_cell_indices,
      cv::Mat &depth_map
    );

    void removeMaskBoundary(
      cv::Mat& mask,
      int border_size
    );

    // Intrinsics of the camera
    PinholeCameraModel dvs_cam_;
    Eigen::Matrix3d K_;
    int width_;
    int height_;
    std::stringstream distortion_model_;

    // (Constant) parameters that define the DSI (size and intrinsics)
    ShapeDSI dsi_shape_;
    PinholeCameraModel virtual_cam_;

    // Precomputed vector of num_depth_cells_ inverse depths,
    // uniformly sampled in inverse depth space
    TypeDepthVector depths_vec_;
    std::vector<float> raw_depths_vec_;

    // Precomputed (normalized) bearing vectors for each pixel of the reference image
    Eigen::Matrix2Xd precomputed_rectified_points_;

    const size_t packet_size_ = 1024;

  };
}

#endif