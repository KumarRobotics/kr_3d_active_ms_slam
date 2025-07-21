#pragma once

#include <cubeFactor.h>
#include <cylinderFactor.h>
#include <gtsam/geometry/BearingRange.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sam/BearingFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <boost/array.hpp>
#include <string.h>
#include <fstream>

// using namespace std;
using namespace gtsam;
using namespace gtsam_cylinder;
using namespace gtsam_cube;

// using gtsam_cube::CubeFactor;
// using gtsam_cube::CubeMeasurement;

// define a symbol for each type of nodes in the graph

// using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::C;  // Cube Landmark
using symbol_shorthand::L;  // (Cylinder) Landmark
using symbol_shorthand::U;  // (UGV) Landmark
// using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
// pose means tf_sensor_to_map, instead of tf_map_to_sensor
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::Y;  // Pose3 (x,y,z,r,p,y)

// bearing factor (ref:
// http://docs.ros.org/en/melodic/api/gtsam/html/testBearingFactor_8cpp_source.html)
typedef BearingFactor<Pose3, Point3> BearingFactor3D;
// bearing-range factor
// (ref:https://bitbucket.org/gtborg/gtsam/issues/296/having-trouble-using-bearingrangefactor3d)
typedef BearingRangeFactor<Pose3, Point3> BearingRangeFactor3D;

// This class wraps the graph optimization back-end.
// To add application specific knowledge, such as "add odom measurement every x
// meters" it is recommended that you inherit this class (e.g. GSLAMNode)
class SemanticFactorGraph {
 public:
  explicit SemanticFactorGraph();

  void setPriors(const Pose3& pose_prior, const int& robotID);
  void solve();
  // factors
  // TODO: make the counter correct
  void addGPSFactor(const Point3& gps_measurement);
  void addCylinderFactor(const size_t poseIdx, const size_t cylIdx,
                         const Pose3& pose, const CylinderMeasurement& cylinder,
                         bool alreadyExists, const int& robotID);
  void addBearingFactor(const size_t poseIdx, const size_t ugvIdx,
                        const Point3& bearing_measurement,
                        const double& range_measurement);
  void addCubeFactor(const size_t poseIdx, const size_t cubeIdx,
                     const Pose3& pose, const CubeMeasurement& cube_global_meas,
                     bool alreadyExists, const int& robotID);
  void addKeyPoseAndBetween(
      const size_t fromIdx, const size_t toIdx, const Pose3& relativeMotion,
      const Pose3& poseEstimate, const int& robotID,
      boost::optional<boost::array<double, 36>> cov = boost::none,
      const bool& loopClosureFound = false,
      const SE3& loop_closure_pose = SE3(),
      const size_t& closure_matched_pose_idx = 0);
  void addLoopClosureFactor(const Pose3 poseRelative, const size_t fromIdx,
                            const size_t toIdx);
  void addPointLandmarkKey(const size_t ugvIdx, const Point3& ugv_position);

  // getters and setters
  bool getPose(const size_t idx, const int& robotID, Pose3& poseOut);
  Eigen::MatrixXd getPoseCovariance(const int idx, const int& robotID);
  CylinderMeasurement getCylinder(const int idx);
  CubeMeasurement getCube(const int idx);
  Point3 getPointLandmark(const int idx);
  // Pose3 getCurrentPose();
  Pose3 getOdomToGPSTF();
  void setOdomToGPSTF(const Pose3& pose);
  double estimateClosureInfoGain(
      const std::vector<size_t>& candidateTrajPoseIndices,
      const std::vector<double>& travel_distances);
  int addFakeClosureFactor(const size_t& currentPoseIdx,
                           const size_t& histroyPoseIdx,
                           const double& travel_distance);

  void logEntropy();

  // these two noise models will be passed as parameters from graphWarpper
  boost::shared_ptr<noiseModel::Diagonal> noise_model_pose;
  Vector6 noise_model_pose_vec_per_m;
  SharedNoiseModel noise_model_bearing;
  double noise_model_pose_inflation = 0.0;
  double start_timestamp = 0.0;

  boost::shared_ptr<noiseModel::Diagonal> noise_model_closure;

 protected:
  // Noise models
  boost::shared_ptr<noiseModel::Diagonal> noise_model_prior_first_pose;
  boost::shared_ptr<noiseModel::Diagonal> noise_model_gps;
  boost::shared_ptr<noiseModel::Diagonal> noise_model_cylinder;
  boost::shared_ptr<noiseModel::Diagonal> noise_model_cube;

  // Aux attributes
//   Pose3 current_pose_global_;
  Pose3 odomToGPS_;

  // Graph factors and values
  NonlinearFactorGraph fgraph;
  Values fvalues;
  Values currEstimate;
  // ISAM Parameters
  ISAM2Params isam_params, isam_params_loop;
  ISAM2 *isam, *isam_loop;
  size_t latest_to_idx_ = 0;
  size_t latest_landmark_counter = 0;

  // TODO: these are not used for now
  size_t fake_loop_counter = 0;
  NonlinearFactorGraph fgraph_loop;
  Values fvalues_loop;

  ros::Time start_time_;

};