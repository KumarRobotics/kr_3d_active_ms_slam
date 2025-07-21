#include <graphWrapper.h>

SemanticFactorGraphWrapper::SemanticFactorGraphWrapper() {
  cyl_counter_ = 0;
  cube_counter_ = 0;
  pose_counter_robot1_ = 0;
  point_landmark_counter_ = 0;
  pose_counter_robot2_ = 0;
}

// for active team localization
bool SemanticFactorGraphWrapper::addLoopClosureObservation(
    const SE3 &relativeMotionSE3, const SE3 &poseEstimateSE3,
    const boost::array<double, 36> &cov, const SE3 &loop_closure_relative_pose,
    const size_t &closure_matched_pose_idx) {
  // default only have 1 aerial robot for active team localization
  gtsam::Pose3 relativeMotion(relativeMotionSE3.matrix());
  gtsam::Pose3 poseEstimate(poseEstimateSE3.matrix());
  int robotID = 0;

  printf("#### ROBOT pose counter:####\n");
  std::cout << pose_counter_robot1_ << "\n";

  bool optimize = false;

  if (pose_counter_robot1_ == 0) {
    ROS_ERROR_STREAM("+++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    ROS_ERROR_STREAM("First pose should not be the loop closure pose!!!!!!!!!");
    ROS_ERROR_STREAM("+++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    return false;
  } else {
    // add BetweenFactor for two consecutive poses (initialize the new pose node
    // with the curr_pose too)
    bool loopClosureFound = true;

    addKeyPoseAndBetween(pose_counter_robot1_ - 1, pose_counter_robot1_,
                         relativeMotion, poseEstimate, robotID, cov,
                         loopClosureFound, loop_closure_relative_pose,
                         closure_matched_pose_idx);

    pose_counter_robot1_++;

    std::cout << "Running factor-graph optimization!" << '\n';
    solve();
    return true;
  }
}

// for active team localization
bool SemanticFactorGraphWrapper::addOdomBearingObservation(
    const SE3 &relativeMotionSE3, const SE3 &poseEstimateSE3,
    const boost::array<double, 36> &cov,
    const std::vector<Point3> &bearing_factors,
    const std::vector<double> &range_factors, const std::vector<size_t> &ids,
    const std::vector<Point3> &landmark_body_positions, const double &data_association_distance) {
  // default only have 1 aerial robot for active team localization
  gtsam::Pose3 relativeMotion(relativeMotionSE3.matrix());
  gtsam::Pose3 poseEstimate(poseEstimateSE3.matrix());
  int robotID = 0;
  // pose counter will record the key poses being added to the graph, each of
  // which is associated with a set of cylinder and cuboid factors

  printf("#### ROBOT pose counter:####\n");
  std::cout << pose_counter_robot1_ << "\n";

  bool optimize = false;

  if (pose_counter_robot1_ == 0) {
    start_timestamp = ros::Time::now().toSec();
    // set priors for the first pose
    setPriors(poseEstimate, robotID);
    std::cout << "The prior for the robot 1st pose is set as:" << '\n';
    std::cout << poseEstimate.matrix();

  } else {
    // add BetweenFactor for two consecutive poses (initialize the new pose node
    // with the curr_pose too)
    addKeyPoseAndBetween(pose_counter_robot1_ - 1, pose_counter_robot1_,
                         relativeMotion, poseEstimate, robotID, cov);

    optimize = true;
  }
  if ((bearing_factors.size()) != (range_factors.size())) {
    std::cout << "ERROR: bearing and range factors have different sizes!"
              << '\n';
    std::cout << "ERROR: their sizes are: " << bearing_factors.size() << " "
              << range_factors.size() << '\n';
    return false;
  }
  if ((bearing_factors.size()) != (ids.size())) {
    std::cout
        << "ERROR: bearing factors and measurement id have different sizes!"
        << '\n';
    return false;
  }
  if ((bearing_factors.size()) != (landmark_body_positions.size())) {
    std::cout << "ERROR: bearing factors and landmark_body_positions have "
                 "different sizes!"
              << '\n';
    return false;
  }

  // get all landmarks for data assocaition 
  std::vector<SE3> allLandmarks;
  std::vector<size_t> allLandmarkInds;
  getAllLandmarks(allLandmarks, allLandmarkInds);
  // get the positions of all landmarks
  std::vector<Point3> allLandmarksPos;
  for (auto i = 0; i < allLandmarks.size(); i++) {
    allLandmarksPos.push_back(allLandmarks[i].translation());
  }
  // sanity check all landmarkspose and all landarks ids should be the same length
  if (allLandmarksPos.size() != allLandmarkInds.size()) {
    ROS_ERROR_STREAM("ERROR: allLandmarksPos and allLandmarkInds have "
                     "different sizes!");
  }
  

  bool add_range_bearing_factor_only_once = false;
  // iterate through all range-bearing measurements
  // if no factors, will skip automatically
  for (size_t i = 0; i < bearing_factors.size(); i++) {
    // check if ids[i] is the largest id so far
    if (ids[i] > point_landmark_counter_) {
      point_landmark_counter_ = ids[i];
    }
    auto it = centroid_ids_.find((ids[i]));
    if (it == centroid_ids_.end()) {
      // nearest neighbor search to check if the landmark is already in the map
      // If we didn't find the id, use nearest neighbor to find association
      // No matches, add as new landmarks
      // apply transform on the landmark body position to get the world position
      // of the landmark
      gtsam::Point3 landmark_world_position;
      landmark_world_position =
          poseEstimate.transformFrom(landmark_body_positions[i]);

      ROS_INFO_STREAM("landmark body positions are: x"
                       << landmark_body_positions[i].x() << " y"
                       << landmark_body_positions[i].y() << " z"
                       << landmark_body_positions[i].z());
      ROS_INFO_STREAM(
          "verify landmark_world_position of landmarks are correct, currently "
          "they "
          "are: x:"
          << landmark_world_position.x()
          << " y: " << landmark_world_position.y()
          << " z: " << landmark_world_position.z());

      // Iterate through all landmarks to find the nearest neighbor
      bool current_match_found = false;
      for (size_t lmrk_idx = 0; lmrk_idx < allLandmarksPos.size(); lmrk_idx++)
      {
        // calculate the distance between the current landmark and the lmrk_idxth
        // landmark
        double distance = (landmark_world_position - allLandmarksPos[lmrk_idx]).norm();
        // if the distance is smaller than the threshold, we consider it as the
        // same landmark
        if (distance < data_association_distance) {
          if (!add_range_bearing_factor_only_once) {
            optimize = true;
            addBearingFactor(pose_counter_robot1_, allLandmarkInds[lmrk_idx], bearing_factors[i],
                            range_factors[i]);
            centroid_ids_.at(allLandmarkInds[lmrk_idx])++;
          } else {
            ROS_WARN(
                "ONLY ADDING RANGE-BEARING FACTOR ONCE FOR EACH LANDMARK!");
          }
          current_match_found = true;
          break;
        }
      }
      if (!current_match_found) {
        ROS_ERROR_STREAM("Adding prior for landmark # " << ids[i]);

        auto ugv_position = landmark_world_position;
        addPointLandmarkKey(ids[i], ugv_position);
        centroid_ids_.insert(std::make_pair(ids[i], 1));

        optimize = true;
        addBearingFactor(pose_counter_robot1_, ids[i], bearing_factors[i],
                        range_factors[i]);
      }
    } else {
      if (!add_range_bearing_factor_only_once) {
        optimize = true;
        addBearingFactor(pose_counter_robot1_, ids[i], bearing_factors[i],
                        range_factors[i]);
        // update counter
        centroid_ids_.at(ids[i])++;
        std::cout << "Bearing measurement is: " << bearing_factors[i] << '\n';
        std::cout << "Range measurement is: " << range_factors[i] << '\n';
        std::cout << "Landmark id is: " << ids[i] << '\n';
        std::cout << "Landmark counter is: " << centroid_ids_.at(ids[i]) << '\n';
      } else {
            ROS_WARN(
                "ONLY ADDING RANGE-BEARING FACTOR ONCE FOR EACH LANDMARK!");
          }
    }
  }

  // This recording of previous_pose is very confusing and conflict with the
  // recording in the warpper function that uses this class, therefore we are no
  // longer use this, we directly pass in relativeMotion from the wrapper
  // function

  pose_counter_robot1_++;

  if (optimize) {
    solve();
    return true;
  }
  return false;
}

// for generic SLOAM (called in sloamNode.cpp)
bool SemanticFactorGraphWrapper::addSLOAMObservation(
    const MapManager &semanticMap, const CubeMapManager &cube_semantic_map,
    const std::vector<int> &cyl_matches, const std::vector<Cylinder> &cylinders,
    const std::vector<int> &cube_matches,
    const std::vector<Cube> &scan_cubes_world, const SE3 &relativeMotionSE3,
    const SE3 &poseEstimateSE3, const int &robotID) {
  // pose counter will record the key poses being added to the graph, each of
  // which is associated with a set of cylinder and cuboid factors
  size_t pose_counter;

  gtsam::Pose3 curr_pose(poseEstimateSE3.matrix());
  gtsam::Pose3 relativeMotion(relativeMotionSE3.matrix());

  if (robotID == 0) {
    pose_counter = pose_counter_robot1_;
  } else if (robotID == 1) {
    pose_counter = pose_counter_robot2_;
  }

  if (pose_counter == 0) {
    // set priors for the first pose
    setPriors(curr_pose, robotID);
  } else {
    // add BetweenFactor for two consecutive poses (initialize the new pose node
    // with the curr_pose too)

    // if cov is not specified, deafult covariance will be used, see
    // noise_model_pose param in graph.cpp
    addKeyPoseAndBetween(pose_counter - 1, pose_counter, relativeMotion,
                         curr_pose, robotID);
  }

  bool optimize = false;
  const auto matchesMap = semanticMap.getMatchesMap();
  const auto cubeMatchesMap = cube_semantic_map.getMatchesMap();
  if (cyl_matches.size() <= 1) {
    std::cout << "Too few cylinders to provide 6DOF constraints!" << '\n';
  }

  for (auto i = 0; i < cyl_matches.size(); i++) {
    if (cyl_matches[i] == -1) {
      // No matches, add as new landmarks
      addCylinderFactor(pose_counter, cyl_counter_, curr_pose, cylinders[i],
                        false, robotID);
      cyl_counter_++;
    } else {
      // transform from observation to map index
      int mapIdx = matchesMap.at(cyl_matches[i]);
      addCylinderFactor(pose_counter, mapIdx, curr_pose, cylinders[i], true,
                        robotID);
      optimize = true;
    }
  }

  for (auto i = 0; i < cube_matches.size(); i++) {
    if (cube_matches[i] == -1) {
      // No matches, add as new landmarks
      addCubeFactor(pose_counter, cube_counter_, curr_pose, scan_cubes_world[i],
                    false, robotID);
      cube_counter_++;
    } else {
      // transform from observation to map index

      int mapIdx = cubeMatchesMap.at(cube_matches[i]);

      addCubeFactor(pose_counter, mapIdx, curr_pose, scan_cubes_world[i], true,
                    robotID);
      optimize = true;
    }
  }

  if (robotID == 0) {
    pose_counter_robot1_ = pose_counter + 1;
  } else if (robotID == 1) {
    pose_counter_robot2_ = pose_counter + 1;
  }

  if (optimize) {
    std::cout << "Running factor-graph optimization!" << '\n';
    solve();
    return true;
  }
  return false;
}

void SemanticFactorGraphWrapper::updateCylinder(
    const CylinderMeasurement &measurement, Cylinder &cyl) {
  cyl.model.root = measurement.root;
  cyl.model.ray = measurement.ray;
  cyl.model.radius = measurement.radius;
}

void SemanticFactorGraphWrapper::updateCube(const CubeMeasurement &measurement,
                                            Cube &cube) {
  cube.model.scale = measurement.scale;
  cube.model.pose = measurement.pose;
}

// for generic SLOAM
void SemanticFactorGraphWrapper::updateFactorGraphMap(
    MapManager &semanticMap, CubeMapManager &cube_semantic_map) {
  auto &map = semanticMap.getMap();
  auto &cube_map = cube_semantic_map.getMap();
  for (auto i = 0; i < cyl_counter_; ++i) {
    updateCylinder(getCylinder(i), map[i]);
  }
  for (auto i = 0; i < cube_counter_; ++i) {
    updateCube(getCube(i), cube_map[i]);
  }
}

void SemanticFactorGraphWrapper::getCurrPose(
    SE3 &curr_pose, const int &robotID,
    boost::optional<Eigen::MatrixXd &> cov) {
  size_t pose_counter;
  if (robotID == 0) {
    pose_counter = pose_counter_robot1_;
  } else if (robotID == 1) {
    pose_counter = pose_counter_robot2_;
  }
  gtsam::Pose3 pose;

  bool pose_valid = getPose(pose_counter - 1, robotID, pose);
  curr_pose = SE3(pose.matrix());
  if (!pose_valid) {
    ROS_ERROR_STREAM(
        "getCurrPose fail to fetch pose for pose_idx : " << pose_counter - 1);
  }
  if (cov) {
    *cov = getPoseCovariance(pose_counter - 1, robotID);
  }
}

bool SemanticFactorGraphWrapper::getPoseByID(SE3 &curr_pose,
                                             const int &poseID) {
  gtsam::Pose3 pose;
  bool pose_valid = getPose(poseID, 0, pose);
  curr_pose = SE3(pose.matrix());
  if (!pose_valid) {
    ROS_ERROR_STREAM(
        "getPoseByID fail to fetch pose for pose_idx : " << poseID);
    return false;
  } else {
    return true;
  }
}

void SemanticFactorGraphWrapper::getAllPoses(std::vector<SE3> &optimized_poses,
                                             std::vector<size_t> &pose_inds) {
  for (size_t i = 0; i < pose_counter_robot1_; i++) {
    gtsam::Pose3 pose;
    bool pose_valid = getPose(i, 0, pose);
    if (!pose_valid) {
      ROS_ERROR_STREAM("getAllPoses fail to fetch pose for pose_idx: " << i);
    } else {
      optimized_poses.push_back(SE3(pose.matrix()));
      pose_inds.push_back(i);
    }
  }

  // sanity check
  if (pose_inds.size() != optimized_poses.size()) {
    ROS_ERROR_STREAM(
        "getAllPoses fail to due to pose_inds and optimized_poses having "
        "different sizes!!!");
  }
}

void SemanticFactorGraphWrapper::getAllLandmarks(
    std::vector<SE3> &optimized_landmark_pos, std::vector<size_t> &landmark_inds) {
  ROS_INFO_STREAM("point_landmark_counter_ is: " << point_landmark_counter_);
  for (auto i = 0; i < point_landmark_counter_; i++) {
    gtsam::Point3 landmark_position = getPointLandmark(i);
    if (landmark_position.x() == gtsam::Point3().x() &&
        landmark_position.y() == gtsam::Point3().y() &&
        landmark_position.z() == gtsam::Point3().z()) {
      ROS_INFO_STREAM("fail to fetch landmark idx: " << i);
    } else {
      ROS_INFO_STREAM("landmark position for landmark idx: " << i
                                                              << " is: "
                                                              << landmark_position);
      gtsam::Pose3 landmark_pose =
          gtsam::Pose3(gtsam::Rot3(), landmark_position);
      optimized_landmark_pos.push_back(SE3(landmark_pose.matrix()));
      landmark_inds.push_back(i);
    }
  }
}


void SemanticFactorGraphWrapper::getAllLandmarks(
    std::vector<SE3> &optimized_landmark_pos) {
  // print point_landmark_counter_
  ROS_INFO_STREAM("point_landmark_counter_ is: " << point_landmark_counter_);
  for (auto i = 0; i < point_landmark_counter_; i++) {
    gtsam::Point3 landmark_position = getPointLandmark(i);
    if (landmark_position.x() == gtsam::Point3().x() &&
        landmark_position.y() == gtsam::Point3().y() &&
        landmark_position.z() == gtsam::Point3().z()) {
      ROS_INFO_STREAM("fail to fetch landmark idx: " << i);
    } else {
      // print landmark position got for i, print its position
      ROS_INFO_STREAM("landmark position for landmark idx: " << i
                                                              << " is: "
                                                              << landmark_position);
      gtsam::Pose3 landmark_pose =
          gtsam::Pose3(gtsam::Rot3(), landmark_position);
      optimized_landmark_pos.push_back(SE3(landmark_pose.matrix()));
    }
  }
}