#pragma once

// ROS
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// pcl
#include <cube.h>
#include <cubeMapManager.h>
#include <definitions.h>
#include <graphWrapper.h>
#include <inference.h>
#include <mapManager.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <sloam.h>
#include <sloam_msgs/ROSObservation.h>
#include <tf2/buffer_core.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <trellis.h>
#include <utils.h>
#include <vizTools.h>

#include <loopclosure.hpp>
#include <mutex>
#include <queue>
#include <random>
#include <thread>

namespace sloam {
class SLOAMNode : public sloam {
 public:
  explicit SLOAMNode(const ros::NodeHandle &nh);
  SLOAMNode(const SLOAMNode &) = delete;
  SLOAMNode operator=(const SLOAMNode &) = delete;
  ~SLOAMNode();
  using Ptr = boost::shared_ptr<SLOAMNode>;
  using ConstPtr = boost::shared_ptr<const SLOAMNode>;
  bool runSLOAMNode(const SE3 relativeMotion, const SE3 prevKeyPose,
                    CloudT::Ptr treeCloud, CloudT::Ptr groundCloud,
                    std::vector<Cube> cubesBody, ros::Time stamp, SE3 &outPose,
                    const int &robotID);

 private:
  ros::Publisher groundPub_;

  void initParams_();
  Cloud::Ptr trellisCloud(
      const std::vector<std::vector<TreeVertex>> &landmarks);
  void publishMap_(const ros::Time stamp);
  void publishCubeMaps_(const ros::Time stamp);

  bool prepareInputs_(const SE3 relativeMotion, const SE3 prevKeyPose,
                      CloudT::Ptr tree_cloud, CloudT::Ptr ground_cloud,
                      SloamInput &sloamIn);
  void publishResults_(const SloamInput &sloamIn, const SloamOutput &sloamOut,
                       ros::Time stamp, const int &robotID);

  /*
   * --------------- Visualization ------------------
   */
  ros::NodeHandle nh_;

  tf::TransformBroadcaster worldTfBr_;
  // SLOAM Output Publishers
  ros::Publisher pubMapPose_;
  ros::Publisher pubObs_;

  // DEBUG TOPICS
  // ros::Publisher pubMapTreeFeatures_;
  ros::Publisher pubRobot1Trajectory_;
  ros::Publisher pubRobot2Trajectory_;
  ros::Publisher pubMapGroundFeatures_;
  ros::Publisher pubObsTreeFeatures_;
  ros::Publisher pubObsGroundFeatures_;
  ros::Publisher pubMapTreeModel_;
  ros::Publisher pubSubmapTreeModel_;
  ros::Publisher pubObsTreeModel_;
  ros::Publisher pubMapGroundModel_;
  ros::Publisher pubObsGroundModel_;

  // cuboids related publishers
  ros::Publisher pubMapCubeModel_;
  ros::Publisher pubSubmapCubeModel_;

  // Transform
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  bool static_transforms_initialized_ = false;

  // if point clouds is staggered, do destagger
  bool do_destagger_ = true;

  std::string world_frame_id_, robot_frame_id_, map_frame_id_;

  // Submodule objects
  boost::shared_ptr<seg::Segmentation> segmentator_ = nullptr;
  boost::shared_ptr<loop::UrquhartLoopCloser> loop_closer_ = nullptr;

  Instance graphDetector_;
  MapManager semanticMap_;
  SemanticFactorGraphWrapper factorGraph_;
  FeatureModelParams fmParams_;

  std::thread loopthread_;
  int lastLoopAttemptPose_;
  std::mutex semanticMapMtx_;
  std::mutex loopMtx_;
  std::queue<loop::LoopFactorData> loopQueue_;
  // std::mutex factorGraphMtx_;

  // For cuboid semantic landmarks
  CubeMapManager cube_semantic_map_;
  std::vector<int> cube_matches_;
  std::vector<Cube> submap_cubes_;
  std::vector<Cube> scan_cubes_world_;

  bool global_cube_map_published_;
  bool cube_map_initialized_ = false;
  std::mutex cube_semantic_map_mtx_;
  int counter_for_noise = 0;

  bool firstScan_;
  bool debugMode_;
};
}  // namespace sloam
