#include <pcl/common/io.h>
#include <ros/console.h>
#include <sloamNode.h>

#include <chrono>

namespace sloam {
SLOAMNode::SLOAMNode(const ros::NodeHandle &nh) : nh_(nh) {
  ROS_DEBUG_STREAM("Defining topics" << std::endl);

  // ######################################################################
  // Fake cuboids removed and replaced by cuboids published by process node
  // ######################################################################

  global_cube_map_published_ = False;

  debugMode_ = nh_.param("debug_mode", false);
  if (debugMode_) {
    ROS_DEBUG_STREAM("Running SLOAM in Debug Mode" << std::endl);
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  } else {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Info)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  }

  // Debugging Publishers
  pubMapGroundFeatures_ = nh_.advertise<CloudT>("debug/map_ground_features", 1);
  pubObsTreeFeatures_ = nh_.advertise<CloudT>("debug/obs_tree_features", 1);
  pubObsGroundFeatures_ = nh_.advertise<CloudT>("debug/obs_ground_features", 1);
  pubRobot1Trajectory_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "debug/robot1/trajectory", 1, true);
  pubRobot2Trajectory_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "debug/robot2/trajectory", 1, true);
  pubMapTreeModel_ =
      nh_.advertise<visualization_msgs::MarkerArray>("map", 1, true);
  pubSubmapTreeModel_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "map_cylinder_models", 1, true);
  pubObsTreeModel_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "debug/obs_tree_models", 1, true);
  pubMapGroundModel_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "debug/map_ground_model", 1);
  pubObsGroundModel_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "debug/obs_ground_model", 1);

  groundPub_ = nh_.advertise<CloudT>("segmentation/ground", 1);

  // cuboids related publishers
  pubMapCubeModel_ =
      nh_.advertise<visualization_msgs::MarkerArray>("cubes_map", 1, true);
  pubSubmapCubeModel_ =
      nh_.advertise<visualization_msgs::MarkerArray>("cubes_submap", 1, true);

  // SLOAM publishers
  pubObs_ = nh_.advertise<sloam_msgs::ROSObservation>("observation", 10);
  pubMapPose_ = nh_.advertise<geometry_msgs::PoseStamped>("map_pose", 10);

  ROS_DEBUG_STREAM("Init params" << std::endl);
  firstScan_ = true;
  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));
  initParams_();

  // Loop Closure
  auto temp_loop = boost::make_shared<loop::UrquhartLoopCloser>(10, 2.5);
  loop_closer_ = std::move(temp_loop);
  lastLoopAttemptPose_ = -1;
}

SLOAMNode::~SLOAMNode() {
  if (loopthread_.joinable()) loopthread_.join();
}

void SLOAMNode::initParams_() {
  // PARAMETERS
  float beam_cluster_threshold = nh_.param("beam_cluster_threshold", 0.1);
  int min_vertex_size = nh_.param("min_vertex_size", 2);
  float max_dist_to_centroid = nh_.param("max_dist_to_centroid", 0.2);
  int min_landmark_size = nh_.param("min_landmark_size", 4);
  float min_landmark_height = nh_.param("min_landmark_height", 1);

  Instance::Params params;
  params.beam_cluster_threshold = beam_cluster_threshold;
  params.min_vertex_size = min_vertex_size;
  params.max_dist_to_centroid = max_dist_to_centroid;
  params.min_landmark_size = min_landmark_size;
  params.min_landmark_height = min_landmark_height;

  // Creating objects for submodules
  std::string modelFilepath;
  bool runSegmentation;
  nh_.param<std::string>("seg_model_path", modelFilepath, "");
  nh_.param<bool>("run_segmentation", runSegmentation, true);
  ROS_DEBUG_STREAM("MODEL PATH: " << modelFilepath);
  float fov = nh_.param("seg_lidar_fov", 22.5);
  int lidar_w = nh_.param("seg_lidar_w", 2048);
  int lidar_h = nh_.param("seg_lidar_h", 64);
  do_destagger_ = nh_.param("do_destagger", true);
  auto temp_seg = boost::make_shared<seg::Segmentation>(
      modelFilepath, fov, -fov, lidar_w, lidar_h, 1, do_destagger_,
      runSegmentation);
  segmentator_ = std::move(temp_seg);

  semanticMap_ = MapManager();
  cube_semantic_map_ = CubeMapManager();
  factorGraph_ = SemanticFactorGraphWrapper();

  graphDetector_ = Instance();
  graphDetector_.set_params(params);
  graphDetector_.reset_tree_id();

  fmParams_.scansPerSweep = 1;
  fmParams_.minTreeModels = nh_.param("min_tree_models", 15);
  fmParams_.minGroundModels = nh_.param("min_ground_models", 50);
  fmParams_.maxLidarDist = nh_.param("max_lidar_dist", 15.0);
  fmParams_.maxGroundLidarDist = nh_.param("max_ground_dist", 30.0);
  fmParams_.minGroundLidarDist = nh_.param("min_ground_dist", 0.0);

  fmParams_.twoStepOptim = nh_.param("two_step_optim", false);

  fmParams_.groundRadiiBins = nh_.param("ground_radii_bins", 5);
  fmParams_.groundThetaBins = nh_.param("ground_theta_bins", 36);
  fmParams_.groundMatchThresh = nh_.param("ground_match_thresh", 2.0);
  fmParams_.groundRetainThresh = nh_.param("ground_retain_thresh", 2.0);

  fmParams_.maxTreeRadius = nh_.param("max_tree_radius", 1.0);
  fmParams_.maxAxisTheta = nh_.param("max_axis_theta", 10.0);  // in degrees
  fmParams_.maxFocusOutlierDistance = 0.5;
  fmParams_.roughTreeMatchThresh = nh_.param("rough_tree_match_thresh", 3.0);
  fmParams_.treeMatchThresh = nh_.param("tree_match_thresh", 1.0);

  fmParams_.AddNewTreeThreshDist = nh_.param("add_new_tree_thresh_dist", 2.0);

  fmParams_.featuresPerTree = nh_.param("features_per_tree", 2);
  fmParams_.numGroundFeatures = nh_.param("num_ground_features", 10);

  fmParams_.defaultTreeRadius = nh_.param("default_tree_radius", 0.1);

  // Mapper
  setFmParams(fmParams_);

  // Frame Ids
  nh_.param<std::string>("world_frame_id", world_frame_id_, "world");
  nh_.param<std::string>("map_frame_id", map_frame_id_, "map");
  nh_.param<std::string>("robot_frame_id", robot_frame_id_, "robot");
  ROS_DEBUG_STREAM("WORLD FRAME " << world_frame_id_);
  ROS_DEBUG_STREAM("MAP FRAME " << map_frame_id_);
  ROS_DEBUG_STREAM("ROBOT FRAME " << robot_frame_id_);
}

void SLOAMNode::publishMap_(const ros::Time stamp) {
  sloam_msgs::ROSObservation obs;
  obs.header.stamp = stamp;
  obs.header.frame_id = map_frame_id_;
  if (pubMapTreeModel_.getNumSubscribers() > 0) {
    auto semantic_map = semanticMap_.getVizMap();
    // viz only
    visualization_msgs::MarkerArray mapTMarkerArray;
    size_t cid = 500000;
    vizTreeModels(semantic_map, mapTMarkerArray, cid);
    pubMapTreeModel_.publish(mapTMarkerArray);
  }
}

void SLOAMNode::publishCubeMaps_(const ros::Time stamp) {
  sloam_msgs::ROSObservation obs;
  obs.header.stamp = stamp;
  obs.header.frame_id = map_frame_id_;
  // get the cubes that are observed more than once
  auto semantic_map = cube_semantic_map_.getVizMap();
  visualization_msgs::MarkerArray cubeMapTMarkerArray;
  size_t cube_id = 0;
  // publish all cubes that have been observed more than once
  vizCubeModels(semantic_map, cubeMapTMarkerArray, cube_id, True);
  pubMapCubeModel_.publish(cubeMapTMarkerArray);
  global_cube_map_published_ = True;  // delete this if not useful
  visualization_msgs::MarkerArray cubeSubMapTMarkerArray;
  // publish current scan cube map
  vizCubeModels(scan_cubes_world_, cubeSubMapTMarkerArray, cube_id, False);
  pubSubmapCubeModel_.publish(cubeSubMapTMarkerArray);
}

Cloud::Ptr SLOAMNode::trellisCloud(
    const std::vector<std::vector<TreeVertex>> &landmarks) {
  CloudT::Ptr vtxCloud = CloudT::Ptr(new CloudT);
  std::vector<float> color_values((int)landmarks.size());
  std::iota(std::begin(color_values), std::end(color_values), 1);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::shuffle(color_values.begin(), color_values.end(), gen);
  int color_id = 0;

  for (auto landmark : landmarks) {
    for (auto vtx : landmark) {
      for (auto point : vtx.points) {
        point.intensity = color_values[color_id];
        vtxCloud->points.push_back(point);
      }
    }
    color_id++;
  }
  vtxCloud->height = 1;
  vtxCloud->width = vtxCloud->points.size();
  vtxCloud->header.frame_id = map_frame_id_;
  return vtxCloud;
}

bool SLOAMNode::prepareInputs_(const SE3 relativeMotion, const SE3 prevKeyPose,
                               CloudT::Ptr rawTreeCloud,
                               CloudT::Ptr rawGroundCloud,
                               SloamInput &sloamIn) {
  // destagger
  Cloud::Ptr destaggeredTreeCloud(new pcl::PointCloud<Point>);
  pcl::copyPointCloud(*rawTreeCloud, *destaggeredTreeCloud);
  Cloud::Ptr destaggeredGroundCloud(new pcl::PointCloud<Point>);
  pcl::copyPointCloud(*rawGroundCloud, *destaggeredGroundCloud);
  if (do_destagger_) {
    segmentator_->destaggerCloud(rawTreeCloud, destaggeredTreeCloud);
    segmentator_->destaggerCloud(rawGroundCloud, destaggeredGroundCloud);
    ROS_INFO_THROTTLE(1, "SLOAM is destaggering point cloud...");
  }

  SE3 poseEstimate = prevKeyPose * relativeMotion;
  std::cout << "prevKeyPose estimate 2: " << prevKeyPose.matrix() << "\n";
  std::cout << "relativeMotion estimate 2: " << relativeMotion.matrix() << "\n";
  std::cout << "poseEstimate estimate 2: " << poseEstimate.matrix() << "\n";

  sloamIn.poseEstimate = poseEstimate;
  sloamIn.distance = relativeMotion.translation().norm();

  // get K closest landmarks as submap, this step will update the index matches
  // between submap and global map, i.e., matchesMap_
  semanticMap_.getSubmap(poseEstimate, sloamIn.mapModels);

  // get K closest landmarks (from cube_models_) as submap, this step will
  // generate matchesMap_, i.e., the index matches between submap and global map
  cube_semantic_map_.getSubmap(poseEstimate, submap_cubes_);

  // ######################################################################
  // Fake cuboids removed and replaced by cuboids published by process node
  // ######################################################################

  if (!firstScan_ && sloamIn.mapModels.size() == 0) {
    ROS_DEBUG("Discarding msg");
    return false;
  }

  // RUN SEGMENTATION
  // ground cloud is expected in the body frame
  sloamIn.groundCloud = destaggeredGroundCloud;
  ROS_DEBUG_STREAM(
      "Num ground features available: " << destaggeredGroundCloud->width);

  // Trellis graph instance segmentation
  graphDetector_.computeGraph(destaggeredTreeCloud, sloamIn.landmarks);

  ROS_INFO_STREAM(
      "Num Landmarks detected with Trellis: " << sloamIn.landmarks.size());
  ROS_INFO_STREAM("Num Map Landmarks: " << sloamIn.mapModels.size());
  return true;
}

void SLOAMNode::publishResults_(const SloamInput &sloamIn,
                                const SloamOutput &sloamOut, ros::Time stamp,
                                const int &robotID) {
  publishMap_(stamp);
  publishCubeMaps_(stamp);

  pubMapPose_.publish(makeROSPose(sloamOut.T_Map_Curr, map_frame_id_, stamp));
  //////////////////////////////////////////////////////
  // FOR DEBUGGING
  // Republish input map
  if (debugMode_) {
    auto trajectory = semanticMap_.getTrajectory(robotID);
    visualization_msgs::MarkerArray trajMarkers = vizTrajectory(trajectory);
    if (robotID == 0) {
      pubRobot1Trajectory_.publish(trajMarkers);
    } else if (robotID == 1) {
      pubRobot2Trajectory_.publish(trajMarkers);
    }

    visualization_msgs::MarkerArray mapTMarkerArray;
    size_t cid = 200000;
    vizTreeModels(sloamIn.mapModels, mapTMarkerArray, cid);
    pubSubmapTreeModel_.publish(mapTMarkerArray);

    // Publish aligned observation
    size_t cylinderId = 100000;
    visualization_msgs::MarkerArray obsTMarkerArray;
    vizTreeModels(sloamOut.tm, obsTMarkerArray, cylinderId);
    pubObsTreeModel_.publish(obsTMarkerArray);

    auto trellis = trellisCloud(sloamIn.landmarks);
    pcl::transformPointCloud(*trellis, *trellis, sloamOut.T_Map_Curr.matrix());
    pubObsTreeFeatures_.publish(trellis);

    // After running sloam, prevGround is already relative to this obs
    auto obsGFeats = getPrevGroundFeatures();
    obsGFeats.header.frame_id = map_frame_id_;
    pubObsGroundFeatures_.publish(obsGFeats);
    pubObsGroundModel_.publish(
        vizGroundModel(getPrevGroundModel(), map_frame_id_, 111));
  }
}


bool SLOAMNode::runSLOAMNode(const SE3 relativeMotion, const SE3 prevKeyPose,
                             CloudT::Ptr treeCloud, CloudT::Ptr groundCloud,
                             std::vector<Cube> cubesBody, ros::Time stamp,
                             SE3 &outPose, const int &robotID) {
  semanticMapMtx_.lock();
  SloamInput sloamIn = SloamInput();

  // prepare inputs does the following:
  // (1) call getSubmap function to obtain the sub-map (a map surrounding the
  // robot) from global map
  // (2) run semantic segmentation and trellis graph to detect objects
  if (!prepareInputs_(relativeMotion, prevKeyPose, treeCloud, groundCloud,
                      sloamIn))
    return false;

  SloamOutput sloamOut = SloamOutput();
  ROS_INFO_STREAM("Entering Callback. Lidar data stamp: " << stamp);

  if (debugMode_ && !firstScan_) {
    auto mapGFeats = getPrevGroundFeatures();
    mapGFeats.header.frame_id = map_frame_id_;
    pubMapGroundFeatures_.publish(mapGFeats);
    pubMapGroundModel_.publish(
        vizGroundModel(getPrevGroundModel(), map_frame_id_, 444));
  }

  // RunSloam does the following:
  // (1) estimating the ground and cylinders, but not doing optimization,
  // (2) associates the landmark detections with sub map landmarks
  // input landmarks are in local frame, output landmarks are in the world frame
  // (see sloam.cpp)
  std::cout << "poseEstimate estimate 3: " << sloamIn.poseEstimate.matrix()
            << "\n";
  bool success = RunSloam(sloamIn, cubesBody, scan_cubes_world_, submap_cubes_,
                          cube_matches_, sloamOut);

  // when optimization is turned off, then sloamOut.T_Map_Curr will be the same
  // as sloamIn.poseEstimate, which is raw odometry pose

  if (!success) {
    semanticMapMtx_.unlock();
    return false;
  }

  // Only update map if RunSloam is successful (see mapManager.cpp)
  semanticMap_.updateMap(sloamOut.T_Map_Curr, sloamOut.tm, sloamOut.matches,
                         robotID);
  // here cubes should be in the global frame (see cubeMapManager.cpp)
  cube_semantic_map_.updateMap(sloamOut.T_Map_Curr, scan_cubes_world_,
                               cube_matches_, robotID);

  if (firstScan_ && sloamOut.tm.size() > 0) firstScan_ = false;

  ROS_INFO("cube map updated");

  // For original SLOAM
  // This step will add cylinder factors into factor graph

  ROS_INFO("adding sloam observation");

  // sanity check
  if (cube_matches_.size() != scan_cubes_world_.size())
    ROS_ERROR_STREAM("# cube matches does not matches # detected cubes!!");
  // here cubes should be in the global frame
  // (see graphWrapper.cpp)

  // Here the  sloamOut.T_Map_Curr will be added as the between factor in the
  // factor graph, it is the same as sloamIn.poseEstimate, which is raw vio
  // odometry pose

  // for generic SLOAM
  // This step will add cylinder and cuboid factors into factor graph
  bool optimized = factorGraph_.addSLOAMObservation(
      semanticMap_, cube_semantic_map_, sloamOut.matches, sloamOut.tm,
      cube_matches_, scan_cubes_world_, relativeMotion, sloamOut.T_Map_Curr,
      robotID);
  ROS_INFO("sloam observation added");

  if (optimized) {
    factorGraph_.updateFactorGraphMap(semanticMap_, cube_semantic_map_);
    factorGraph_.getCurrPose(sloamOut.T_Map_Curr, robotID);
  }

  ROS_DEBUG_STREAM(
      "\n---------- OPTMIZATION POSE OUTPUT -----------------\n"
      << sloamOut.T_Map_Curr.matrix()
      << "\n-----------------------------------------------------\n");

  outPose = sloamOut.T_Map_Curr;
  publishResults_(sloamIn, sloamOut, stamp, robotID);
  semanticMapMtx_.unlock();

  return success;
}

}  // namespace sloam