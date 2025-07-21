#include <graph.h>
#include <ros/console.h>
#include <ros/ros.h>

SemanticFactorGraph::SemanticFactorGraph() {
  isam_params.factorization = ISAM2Params::CHOLESKY;
  isam_params.relinearizeSkip = 1;
  isam_params.relinearizeThreshold = 0.1;
  isam = new ISAM2(isam_params);

  // do the same with isam_loop and isam_params_loop
  isam_params_loop.factorization = ISAM2Params::CHOLESKY;
  isam_params_loop.relinearizeSkip = 1;
  isam_params_loop.relinearizeThreshold = 0.1;
  isam_loop = new ISAM2(isam_params_loop);

  // VERY IMPORTANT: the first three corresponds to Rotation and the last three
  // corresponds to Translation
  Vector6 noise_vec_prior_first_pose;
  noise_vec_prior_first_pose << 0.000001, 0.000001, 0.000001, 0.00001, 0.00001, 0.00001;
  noise_model_prior_first_pose =
      noiseModel::Diagonal::Sigmas(noise_vec_prior_first_pose);
  // GET THIS FROM GPS SENSOR
  noise_model_gps = noiseModel::Diagonal::Sigmas(Vector3::Ones());
  // Landmark Noise model
  noise_model_cylinder =
      noiseModel::Diagonal::Sigmas(100 * Vector7::Ones() * 4);

  // update the cuboid measurement noise
  double stddev_angle = 100 * 0.05;  // 120.0 * (3.14 / 180);
  double stddev_pos = 100 * 0.005;   // 3.0;
  Vector9 noise_vec;
  noise_vec << stddev_angle, stddev_angle, stddev_angle, stddev_pos, stddev_pos,
      stddev_pos, stddev_pos, stddev_pos, stddev_pos;
  noise_model_cube = noiseModel::Diagonal::Sigmas(noise_vec);


  // ###########IMPORTANT: these will be reset in factorGraphATL.cpp ######
  // (search factorGraph_.noise_model_pose you will find where it is reset)
  noise_model_pose_vec_per_m = Vector6::Ones();
  noise_model_pose = noiseModel::Diagonal::Sigmas(Vector6::Ones());
  noise_model_closure = noiseModel::Diagonal::Sigmas(Vector6::Ones());
  // #######################################################################

  // ##################### IMPORTANT PARAMS for ATL ###################
  double bearing_noise_std_temp = 1;
  noise_model_bearing = noiseModel::Isotropic::Sigma(3, bearing_noise_std_temp);

  start_time_ = ros::Time::now();
}

void SemanticFactorGraph::setPriors(const Pose3 &pose_prior,
                                    const int &robotID) {
  // initial prior based on GPS
  if (robotID == 0) {
    fgraph.addPrior<Pose3>(X(0), pose_prior, noise_model_prior_first_pose);
    fgraph_loop.addPrior<Pose3>(X(0), pose_prior, noise_model_prior_first_pose);
    // here the insert also set the initial estimate for the optimization
    fvalues.insert(X(0), pose_prior);
    fvalues_loop.insert(X(0), pose_prior);

    printf(
        "################ PRIOR ON FIRST POSE FOR 1ST ROBOT  INCLUDED "
        "################\n");
  }
}

Pose3 SemanticFactorGraph::getOdomToGPSTF() { return odomToGPS_; }

void SemanticFactorGraph::setOdomToGPSTF(const Pose3 &pose) {
  odomToGPS_ = pose;
}


void SemanticFactorGraph::addKeyPoseAndBetween(
    const size_t fromIdx, const size_t toIdx, const Pose3 &relativeMotion,
    const Pose3 &poseEstimate, const int &robotID,
    boost::optional<boost::array<double, 36>> cov, const bool &loopClosureFound,
    const SE3 &loop_closure_relative_pose,
    const size_t &closure_matched_pose_idx) {

  ROS_INFO_STREAM_THROTTLE(
      1,
      "Using hard-coded between-factor covariance instead of "
      "VIO estimated covariance");
  double mins_elapsed;
  if (start_timestamp == 0.0) {
    mins_elapsed = 0.0;
  } else {
    mins_elapsed = (ros::Time::now().toSec() - start_timestamp) / 60.0;
  }
  if (noise_model_pose_inflation > 0) {
    printf(
        "+++++++ inflating covariance with time, if you are playing a bag, "
        "make sure that /use_sim_time is set as true and play bag with "
        "--clock flag \n");

    printf("odom_factor_noise_std_inflation_per_min is set as: \n");
    std::cout << noise_model_pose_inflation << std::endl;
    printf(
        "number of mins since the first "
        "odometry factor: \n");
    std::cout << mins_elapsed << std::endl;
  }

  Vector6 cur_noise_vec;
  cur_noise_vec = noise_model_pose->sigmas();

  fgraph.add(BetweenFactor<Pose3>(X(fromIdx), X(toIdx), relativeMotion,
                                  noiseModel::Diagonal::Sigmas(cur_noise_vec)));
  
  fgraph_loop.add(BetweenFactor<Pose3>(
      X(fromIdx), X(toIdx),
      relativeMotion, noiseModel::Diagonal::Sigmas(cur_noise_vec)));

  if (latest_to_idx_ < toIdx) {
    latest_to_idx_ = toIdx;
  }

  ROS_INFO_STREAM("add between factor from " << fromIdx << "to" << toIdx
                                              << " with relative pose set as : "
                                              << relativeMotion.matrix());

  if (loopClosureFound) {

    gtsam::Pose3 est_loop_closure_pose;

    // get current estimate of X(closure_matched_pose_idx)
    gtsam::Pose3 est_from_pose;

    if(closure_matched_pose_idx == 0){
      ROS_ERROR("closure_matched_pose_idx is 0, this probably means this param is not correctly set, check!!");
    }
    bool cur_pose_valid = getPose(closure_matched_pose_idx, 0, est_from_pose);
    if (!cur_pose_valid) {
      ROS_ERROR_STREAM(
          "get closure_matched_pose_idx fail to fetch pose for pose_idx : " << closure_matched_pose_idx);
      auto odom_pose_prior_factor_noise = cur_noise_vec * 1.0;
      // here the insert also set the initial estimate for the optimization
      fvalues.insert(X(toIdx), poseEstimate);
      fvalues_loop.insert(X(toIdx), poseEstimate);
      return;
    }

    auto est_pose_matrix = est_from_pose.matrix() * loop_closure_relative_pose.matrix();    
    est_loop_closure_pose = gtsam::Pose3(est_pose_matrix);

    fgraph.add(
        BetweenFactor<Pose3>(X(closure_matched_pose_idx), X(toIdx),
                             gtsam::Pose3(loop_closure_relative_pose.matrix()),
                             noise_model_closure));

    fgraph_loop.add(BetweenFactor<Pose3>(
        X(closure_matched_pose_idx), X(toIdx),
        gtsam::Pose3(loop_closure_relative_pose.matrix()), noise_model_closure));


    if (fake_loop_counter != 0) {
      ROS_ERROR("fake_loop_counter should be 0!!!");
    }

    // here the insert also set the initial estimate for the optimization
    fvalues.insert(X(toIdx), est_loop_closure_pose);
    fvalues_loop.insert(X(toIdx),est_loop_closure_pose);

    ROS_ERROR_STREAM("loop_closure_relative_pose being added");
  } else {
    auto odom_pose_prior_factor_noise = cur_noise_vec * 1.0;
    // here the insert also set the initial estimate for the optimization
    fvalues.insert(X(toIdx), poseEstimate);
    fvalues_loop.insert(X(toIdx), poseEstimate);
  }
}

void SemanticFactorGraph::addPointLandmarkKey(const size_t ugvIdx,
                                       const Point3 &ugv_position) {
  fvalues.insert(U(ugvIdx), ugv_position);
  fvalues_loop.insert(U(ugvIdx), ugv_position);
}

void SemanticFactorGraph::addBearingFactor(const size_t poseIdx,
                                           const size_t ugvIdx,
                                           const Point3 &bearing_measurement,
                                           const double &range_measurement) {
  // add bearing measurement with a covaraince
  Unit3 bearing_measurement3D = Pose3().bearing(bearing_measurement);
  double bearing_noise_std = noise_model_bearing->sigmas()[0];
  ROS_INFO_STREAM_THROTTLE(1,
                           "bearing vector should be expressed in body frame");

  fgraph.add(BearingRangeFactor3D(X(poseIdx), U(ugvIdx), bearing_measurement3D,
                                  range_measurement, noise_model_bearing));
  fgraph_loop.add(BearingRangeFactor3D(X(poseIdx),
                                       U(ugvIdx), bearing_measurement3D,
                                       range_measurement, noise_model_bearing));
  if (latest_landmark_counter < ugvIdx) {
    latest_landmark_counter = ugvIdx;
  }
}

void SemanticFactorGraph::addCylinderFactor(const size_t poseIdx,
                                            const size_t cylIdx,
                                            const Pose3 &pose,
                                            const CylinderMeasurement &cylinder,
                                            bool alreadyExists,
                                            const int &robotID) {
  // pose here is tf_map_to_sensor, not tf_sensor_to_map, that is why
  // we need inverse
  CylinderMeasurement c_local = cylinder.project(pose.inverse());

  if (robotID == 0) {
    // X contains tf_sensor_to_map, not tf_map_to_sensor
    fgraph.add(
        CylinderFactor(X(poseIdx), L(cylIdx), c_local, noise_model_cylinder));
  } else if (robotID == 1) {
    fgraph.add(
        CylinderFactor(Y(poseIdx), L(cylIdx), c_local, noise_model_cylinder));
  }

  if (!alreadyExists) {
    fvalues.insert(L(cylIdx), cylinder);
  }
}

void SemanticFactorGraph::addCubeFactor(
    const size_t poseIdx, const size_t cubeIdx, const Pose3 &pose,
    const CubeMeasurement &cube_global_meas_raw, bool alreadyExists,
    const int &robotID) {
  // pose here is tf_map_to_sensor, not tf_sensor_to_map, that is why
  // we need inverse
  CubeMeasurement cube_global_meas = cube_global_meas_raw;
  CubeMeasurement cube_local_meas = cube_global_meas.project(pose.inverse());

  if (robotID == 0) {
    // X contains tf_sensor_to_map, not tf_map_to_sensor
    fgraph.add(
        CubeFactor(X(poseIdx), C(cubeIdx), cube_local_meas, noise_model_cube));
  } else if (robotID == 1) {
    fgraph.add(
        CubeFactor(Y(poseIdx), C(cubeIdx), cube_local_meas, noise_model_cube));
  }

  if (!alreadyExists) {
    // Initialize with first measurement of cube (in fixed world frame)
    fvalues.insert(C(cubeIdx), cube_global_meas);
  }
}


void SemanticFactorGraph::solve() {

  isam->update(fgraph, fvalues);
  isam_loop->update(fgraph_loop, fvalues_loop);

  // Extract the result/current estimates
  currEstimate = isam->calculateEstimate();

  // // Reset the newFactors and newValues list
  fgraph.resize(0);
  fvalues.clear();
  fgraph_loop.resize(0);
  fvalues_loop.clear();
  printf("\n\n");
}


void SemanticFactorGraph::logEntropy() {
  // keep current covaraince matrix
  double sum_entropy_pose = 0.0;
  double sum_entropy_landmark = 0.0;
  size_t num_valid_poses = 0;
  size_t num_valid_landmarks = 0;
  // iterate through all latest_landmark_counter and latest_to_idx_
  for (size_t i = 0; i < latest_to_idx_; i++) {
    // get the covariance matrix of the current pose
    if (isam->valueExists(X(i)) && isam_loop->valueExists(X(i))) {
    gtsam::Matrix cov = isam->marginalCovariance(X(i));
    sum_entropy_pose += cov.trace();
    num_valid_poses++;
    }
  }

  for (size_t i = 0; i < latest_landmark_counter; i++) {
    // get the covariance matrix of the current pose
    if (isam->valueExists(U(i)) && isam_loop->valueExists(U(i))) {
      gtsam::Matrix cov = isam->marginalCovariance(U(i));
      sum_entropy_landmark += cov.trace();
      num_valid_landmarks++;
    }
  }

  // std::ofstream entropy_log_file("entropy_log.txt", ios::app);
  // if (entropy_log_file.fail()){
  //   cout << "open file error!\n";
  // }
  // ros::Time time_now = ros::Time::now();
  // // write a header for entropy log file if it is empty, header is TIME, ENTROPY_POSE, ENTROPY_LANDMARK, NUM_VALID_POSES
  // if (entropy_log_file.tellp() == 0) {
  //   entropy_log_file << "TIME, ENTROPY_POSE, ENTROPY_LANDMARK, NUM_VALID_POSES, NUM_VALID_LANDMARKS" << std::endl;
  // }
  // entropy_log_file << (time_now-start_time_).toSec() << ", " << sum_entropy_pose << ", " << sum_entropy_landmark << ", " << num_valid_poses << ", " << num_valid_landmarks << std::endl;
  // entropy_log_file.close();

}

double SemanticFactorGraph::estimateClosureInfoGain(
    const std::vector<size_t> &candidateTrajPoseIndices,
    const std::vector<double> &travel_distances) {
  // candidateTrajPoseIndices is a vector of pose indices in the candidate
  // trajecotry from current pose to key pose 1, key pose 2, ..., key pose n

  // travel_distances is a vector of travel distances between each pair of poses

  // assert length of travel_distances is one less than length of
  // candidateTrajPoseIndices
  assert(travel_distances.size() == candidateTrajPoseIndices.size() - 1);

  // safety check
  for (size_t i = 0; i < candidateTrajPoseIndices.size(); i++) {
    size_t thisPoseIdx = candidateTrajPoseIndices[i];
    if (currEstimate.exists(X(thisPoseIdx)) == false) {
      ROS_ERROR_STREAM("current pose index is not in the graph, error!");
      return -1;
    }
  }

  NonlinearFactorGraph fgraph_loop_tmp;
  // iterate through all poses in the candidate trajectory
  for (size_t i = 0; i < candidateTrajPoseIndices.size() - 1; i++) {
    size_t currentPoseIdx = candidateTrajPoseIndices[i];
    size_t keyPoseIdx = candidateTrajPoseIndices[i + 1];
    auto motion_noise = noiseModel::Diagonal::Sigmas(
        noise_model_pose_vec_per_m * travel_distances[i]);

    // add between factor between current pose and history pose
    Pose3 pose = currEstimate.at<Pose3>(X(currentPoseIdx));
    Pose3 pose_history = currEstimate.at<Pose3>(X(keyPoseIdx));
    Pose3 pose_relative = pose_history.between(pose);
    fgraph_loop_tmp.add(BetweenFactor<Pose3>(X(keyPoseIdx), X(currentPoseIdx),
                                             pose_relative, motion_noise));
  }

  // add fake loop factor
  ISAM2Result result = isam_loop->update(fgraph_loop_tmp);
  FactorIndices factor_ids = result.newFactorsIndices;

  result.print();

  // keep current covaraince matrix
  double sum_entropy_pose = 0.0;
  double sum_entropy_fake_loop_pose = 0.0;
  double sum_entropy_landmark = 0.0;
  double sum_entropy_fake_loop_landmark = 0.0;
  // iterate through all latest_landmark_counter and latest_to_idx_
  for (size_t i = 0; i < latest_to_idx_; i++) {
    // get the covariance matrix of the current pose
    gtsam::Matrix cov = isam->marginalCovariance(X(i));
    gtsam::Matrix cov_loop = isam_loop->marginalCovariance(X(i));
    sum_entropy_pose += cov.trace();
    sum_entropy_fake_loop_pose += cov_loop.trace();
  }

  for (size_t i = 0; i < latest_landmark_counter; i++) {
    // get the covariance matrix of the current pose
    if (isam->valueExists(U(i)) && isam_loop->valueExists(U(i))) {
      gtsam::Matrix cov = isam->marginalCovariance(U(i));
      gtsam::Matrix cov_loop = isam_loop->marginalCovariance(U(i));
      sum_entropy_landmark += cov.trace();
      sum_entropy_fake_loop_landmark += cov_loop.trace();
    }
  }

  double info_gain_pose = sum_entropy_pose - sum_entropy_fake_loop_pose;
  double info_gain_landmark =
      sum_entropy_landmark - sum_entropy_fake_loop_landmark;

  ROS_INFO_STREAM("sum_entropy_pose: " << sum_entropy_pose);
  ROS_INFO_STREAM("sum_entropy_fake_loop_pose: " << sum_entropy_fake_loop_pose);
  ROS_INFO_STREAM("sum_entropy_landmark: " << sum_entropy_landmark);
  ROS_INFO_STREAM("sum_entropy_fake_loop_landmark: " << sum_entropy_fake_loop_landmark);
  ROS_INFO_STREAM("info_gain_pose: " << info_gain_pose);
  ROS_INFO_STREAM("info_gain_landmark: " << info_gain_landmark);

  // remove fake loop factor
  ISAM2Result result_after_delete =
      isam_loop->update(NonlinearFactorGraph(), Values(), factor_ids);

  bool sanity_check = true;
  // sanity check the covariance matrix
  if (sanity_check) {
    // keep current covaraince matrix
    sum_entropy_pose = 0.0;
    sum_entropy_fake_loop_pose = 0.0;
    sum_entropy_landmark = 0.0;
    sum_entropy_fake_loop_landmark = 0.0;
    // iterate through all latest_landmark_counter and latest_to_idx_
    for (size_t i = 0; i < latest_to_idx_; i++) {
      // get the covariance matrix of the current pose
      if (isam->valueExists(X(i)) && isam_loop->valueExists(X(i))) {
        gtsam::Matrix cov = isam->marginalCovariance(X(i));
        gtsam::Matrix cov_loop = isam_loop->marginalCovariance(X(i));
      sum_entropy_pose += cov.trace();
      sum_entropy_fake_loop_pose += cov_loop.trace();
      }
    }

    for (size_t i = 0; i < latest_landmark_counter; i++) {
      // get the covariance matrix of the current pose
      if (isam->valueExists(U(i)) && isam_loop->valueExists(U(i))) {
        gtsam::Matrix cov = isam->marginalCovariance(U(i));
        gtsam::Matrix cov_loop = isam_loop->marginalCovariance(U(i));
        sum_entropy_landmark += cov.trace();
        sum_entropy_fake_loop_landmark += cov_loop.trace();
      }
    }
    double info_gain_pose = sum_entropy_pose - sum_entropy_fake_loop_pose;
    double info_gain_landmark =
        sum_entropy_landmark - sum_entropy_fake_loop_landmark;
    ROS_INFO_STREAM(
        "++++++++++++++++++++++++++++++++++++++++SANITY "
        "CHECK+++++++++++++++++++++++");
    ROS_INFO_STREAM(
        "info_gain_pose difference (should be 0): " << info_gain_pose);
    ROS_INFO_STREAM(
        "info_gain_landmark difference (should be 0): " << info_gain_landmark);
    ROS_INFO_STREAM(
        "++++++++++++++++++++++++++++++++++++++++SANITY "
        "CHECK+++++++++++++++++++++++");
  }
  // total weighted info gain
  double total_info_gain = 10.0 * info_gain_pose + info_gain_landmark;
  return total_info_gain;
}

CylinderMeasurement SemanticFactorGraph::getCylinder(const int idx) {
  return currEstimate.at<CylinderMeasurement>(L(idx));
}

CubeMeasurement SemanticFactorGraph::getCube(const int idx) {
  return currEstimate.at<CubeMeasurement>(C(idx));
}

Point3 SemanticFactorGraph::getPointLandmark(const int idx) {
  if (isam->valueExists(U(idx))) {
    return currEstimate.at<Point3>(U(idx));
  } else {
    return Point3();
  }
}

bool SemanticFactorGraph::getPose(const size_t idx, const int &robotID,
                                  Pose3 &poseOut) {
  if (robotID == 0) {
    if (isam->valueExists(X(idx))) {
      poseOut = currEstimate.at<Pose3>(X(idx));
      return true;
    } else {
      printf(
          "############# Error!!! Node of interest does not exist in factor "
          "graph #############\n");
      std::cout << "the node is X(" << idx << ")" << std::endl;
      poseOut = Pose3();
      return false;
    }
  } else {
    printf("############# Error: invalid robotID!!! #############\n");
    poseOut = Pose3();
    return false;
  }
}

Eigen::MatrixXd SemanticFactorGraph::getPoseCovariance(const int idx,
                                                       const int &robotID) {
  if (robotID == 0) {
    return isam->marginalCovariance(X(idx));
  } else if (robotID == 1) {
    return isam->marginalCovariance(Y(idx));
  } else {
    printf("############# Error: invalid robotID!!! #############\n");
    return isam->marginalCovariance(X(idx));
  }
}
