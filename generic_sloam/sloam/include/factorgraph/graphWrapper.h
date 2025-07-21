#pragma once

#include <cubeMapManager.h>
#include <definitions.h>
#include <graph.h>
#include <mapManager.h>

#include <boost/array.hpp>

class SemanticFactorGraphWrapper : public SemanticFactorGraph {
 public:
  explicit SemanticFactorGraphWrapper();

  bool addLoopClosureObservation(const SE3 &relativeMotion,
                                 const SE3 &poseEstimate,
                                 const boost::array<double, 36> &cov,
                                 const SE3 &loop_closure_pose,
                                 const size_t &closure_matched_pose_idx);

  bool getPoseByID(SE3 &curr_pose, const int &poseID);


  // data_association_distance: data association of point landmarks, normed distance along X Y Z axis
  bool addOdomBearingObservation(
      const SE3 &relativeMotion, const SE3 &poseEstimate,
      const boost::array<double, 36> &cov,
      const std::vector<Point3> &bearing_factors,
      const std::vector<double> &range_factors, const std::vector<size_t> &ids,
      const std::vector<Point3> &landmark_body_positions, const double& data_association_distance);

  // for generic SLOAM
  bool addSLOAMObservation(const MapManager &semanticMap,
                           const CubeMapManager &cubeSemanticMap,
                           const std::vector<int> &cyl_matches,
                           const std::vector<Cylinder> &cylinders,
                           const std::vector<int> &cube_matches,
                           const std::vector<Cube> &cubes,
                           const SE3 &relativeMotion, const SE3 &poseEstimates,
                           const int &robotID);

  // for original SLOAM
  // void updateFactorGraphMap(MapManager &semanticMap);
  // for generic SLOAM
  void updateFactorGraphMap(MapManager &semanticMap,
                            CubeMapManager &cubeSemanticMap);

  void updateCylinder(const CylinderMeasurement &measurement, Cylinder &cyl);
  void updateCube(const CubeMeasurement &measurement, Cube &cube);

  void getCurrPose(SE3 &curr_pose, const int &robotID,
                   boost::optional<Eigen::MatrixXd &> cov = boost::none);

  void getAllPoses(std::vector<SE3> &optimized_poses,
                   std::vector<size_t> &pose_inds);
  void getAllLandmarks(std::vector<SE3> &optimized_landmark_pos, std::vector<size_t> &landmark_inds);
  void getAllLandmarks(std::vector<SE3> &optimized_landmark_pos);

  size_t pose_counter_robot1_;
  size_t pose_counter_robot2_;

 private:
  size_t cyl_counter_;
  size_t cube_counter_;
  std::unordered_map<size_t, size_t> centroid_ids_;
  size_t point_landmark_counter_;
  gtsam::Pose3 robot1_prev_pose_;
  gtsam::Pose3 robot1_first_pose_;
  gtsam::Pose3 robot2_prev_pose_;
  bool ugv_initialized_;
};