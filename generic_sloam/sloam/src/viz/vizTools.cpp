#include <gtsam/geometry/Quaternion.h>
#include <vizTools.h>

namespace sloam {

// functions adapted from ros tf2/tf2_eigen
geometry_msgs::Quaternion toMsg_(const Quat &in) {
  geometry_msgs::Quaternion msg;
  msg.w = in.w();
  msg.x = in.x();
  msg.y = in.y();
  msg.z = in.z();
  return msg;
}

geometry_msgs::Point toMsg_(const Vector3 &in) {
  geometry_msgs::Point msg;
  msg.x = in.x();
  msg.y = in.y();
  msg.z = in.z();
  return msg;
}

geometry_msgs::Quaternion toRosQuat_(const Sophus::SO3d &R) {
  return toMsg_(R.unit_quaternion());
}

geometry_msgs::Pose toRosPose_(const SE3 &T) {
  geometry_msgs::Pose pose;
  pose.position = toMsg_(T.translation());
  pose.orientation = toRosQuat_(T.so3());
  return pose;
}

nav_msgs::Odometry toRosOdom_(const SE3 &pose, const std::string slam_ref_frame,
                              const ros::Time stamp) {
  nav_msgs::Odometry odom;
  odom.header.frame_id = slam_ref_frame;
  odom.header.stamp = stamp;

  geometry_msgs::Pose rosPose;
  rosPose.position.x = pose.translation()[0];
  rosPose.position.y = pose.translation()[1];
  rosPose.position.z = pose.translation()[2];
  auto quat = pose.unit_quaternion();
  rosPose.orientation.w = quat.w();
  rosPose.orientation.x = quat.x();
  rosPose.orientation.y = quat.y();
  rosPose.orientation.z = quat.z();
  odom.pose.pose = rosPose;
  boost::array<double, 36> cov;
  for (int i = 0; i < 6; i++) {
    double var = 0.0;
    if (i < 3) {
      var = 0.01;
    } else {
      var = 0.01;
    }
    for (int j = 0; j < 6; j++) {
      if (j == i) {
        cov[6 * i + j] = var;
      } else {
        cov[6 * i + j] = 1e-5;
      }
    }
  }
  odom.pose.covariance = cov;
  return odom;
}

visualization_msgs::MarkerArray vizAllLandmarks(const std::vector<SE3> &poses, const std::string &frame_id) {
  visualization_msgs::MarkerArray tMarkerArray;
  visualization_msgs::Marker points;
  int cylinderId = 0;
  // for (auto o : poses) {
  //   geometry_msgs::Point pt;
  //   auto obs_posit = o.translation();
  //   visualization_msgs::Marker marker;
  //   marker.header.frame_id = frame_id;
  //   // marker.header.frame_id = "quadrotor/map";
  //   marker.header.stamp = ros::Time();
  //   marker.id = cylinderId;
  //   marker.type = visualization_msgs::Marker::CYLINDER;
  //   marker.action = visualization_msgs::Marker::ADD;

  //   // height of cylinder
  //   double cyl_height = 1.0;
  //   double cyl_radius = 0.3;

  //   // Center of cylinder
  //   marker.pose.position.x = obs_posit[0];
  //   marker.pose.position.y = obs_posit[1];
  //   marker.pose.position.z = obs_posit[2];

  //   marker.scale.x = cyl_radius * 2;
  //   marker.scale.y = cyl_radius * 2;
  //   marker.scale.z = cyl_height;

  //   marker.color.a = 0.7;
  //   marker.color.r = 0.0;
  //   marker.color.g = 1.0;
  //   marker.color.b = 1.0;

  //   cylinderId++;
  //   tMarkerArray.markers.push_back(marker);
  // }
  // visualize poses as chairs
  for (auto o : poses) {
    geometry_msgs::Point pt;
    auto obs_posit = o.translation();
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    // marker.header.frame_id = "quadrotor/map";
    marker.header.stamp = ros::Time();
    marker.id = cylinderId;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;

    // height of cylinder
    double cyl_height = 1.0;
    double cyl_radius = 0.3;

    // Center of cylinder
    marker.pose.position.x = obs_posit[0];
    marker.pose.position.y = obs_posit[1];
    marker.pose.position.z = obs_posit[2];

    double scale_factor = 1.0;
    marker.scale.x = scale_factor;
    marker.scale.y = scale_factor;
    marker.scale.z = scale_factor;

    marker.color.a = 0.8;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.8;
    // random orientation
    // assign yaw according to x position, every 1 meter of x add 20 degree of yaw
    double yaw = obs_posit[0] * 0.34906585;
    // limit yaw to be within 0-2pi
    yaw = fmod(yaw, 6.28318531);
    // get quaternion from yaw using tf2
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    marker.pose.orientation.x = q[0];
    marker.pose.orientation.y = q[1];
    marker.pose.orientation.z = q[2];
    marker.pose.orientation.w = q[3];
    marker.mesh_resource = "package://sloam/resource/chair.stl";

    cylinderId++;
    tMarkerArray.markers.push_back(marker);
  }
  return tMarkerArray;
}

visualization_msgs::MarkerArray vizTrajectory(const std::vector<SE3> &poses, const std::string &frame_id) {
  visualization_msgs::MarkerArray tMarkerArray;
  visualization_msgs::Marker points, line_strip;
  points.header.frame_id = line_strip.header.frame_id = frame_id;
  points.header.stamp = line_strip.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = "points_and_lines";
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  points.id = 1000;
  line_strip.id = 1001;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.25;
  points.scale.y = 0.25;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the
  // line width
  line_strip.scale.x = 0.1;

  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  // ROS_ERROR_STREAM("Number of poses to be visualized is:" << poses.size());

  for (auto o : poses) {
    // ROS_ERROR_STREAM("poses to be visualized are:" << o.matrix());
    // Between odom line strips
    geometry_msgs::Point pt;
    auto obs_posit = o.translation();
    pt.x = obs_posit[0];
    pt.y = obs_posit[1];
    pt.z = obs_posit[2];
    points.points.push_back(pt);
    line_strip.points.push_back(pt);
  }
  tMarkerArray.markers.push_back(points);
  tMarkerArray.markers.push_back(line_strip);
  return tMarkerArray;
}

visualization_msgs::MarkerArray vizTrajectoryAndPoseInds(
    const std::vector<SE3> &poses, const std::vector<size_t> &pose_inds, const std::string &frame_id) {
  visualization_msgs::MarkerArray tMarkerArray;
  // sanity check
  if (pose_inds.size() != poses.size()) {
    ROS_ERROR_STREAM(
        "vizTrajectoryAndPoseInds fail to due to pose_inds and poses having "
        "different sizes!!!");
    return tMarkerArray;
  }

  for (size_t i = 0; i < poses.size(); i++) {
    auto cur_pose = poses[i];
    auto cur_pose_ind = pose_inds[i];

    // ROS_ERROR_STREAM("poses to be visualized are:" << o.matrix());
    // Between odom line strips
    geometry_msgs::Point pt;
    auto obs_posit = cur_pose.translation();
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "traj_and_pose_inds";
    marker.id = cur_pose_ind;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = obs_posit[0];
    marker.pose.position.y = obs_posit[1];
    marker.pose.position.z = obs_posit[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    tMarkerArray.markers.push_back(marker);
  }

  return tMarkerArray;
}

geometry_msgs::PoseStamped makeROSPose(const SE3 &tf, std::string frame_id,
                                       const ros::Time stamp) {
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = frame_id;
  pose.pose = toRosPose_(tf);
  return pose;
}

SE3 toSE3(geometry_msgs::PoseStamped pose) {
  Vector3d pos;
  Quaterniond quat;
  tf2::fromMsg(pose.pose.position, pos);
  tf2::fromMsg(pose.pose.orientation, quat);

  SE3 tf;
  tf.translation() = pos;
  tf.setQuaternion(quat);

  return tf;
}

void vizTreeModels(const std::vector<Cylinder> &scanTm,
                   visualization_msgs::MarkerArray &tMarkerArray,
                   size_t &cylinderId) {
  for (const auto &tree : scanTm) {
    Scalar maxTreeRadius = 0.25;
    Scalar maxAxisTheta = 45;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "quadrotor/map";
    // marker.header.frame_id = "quadrotor/map";
    marker.header.stamp = ros::Time();
    marker.id = cylinderId;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    // Center of cylinder
    marker.pose.position.x = tree.model.root[0] + 0.5 * tree.model.ray[0];
    marker.pose.position.y = tree.model.root[1] + 0.5 * tree.model.ray[1];
    marker.pose.position.z = 1 + tree.model.root[2] + 0.5 * tree.model.ray[1];
    // marker.pose.position.z = 0;

    // Orientation of cylidner
    Vector3 src_vec(0, 0, 1);
    Quat q_rot = Quat::FromTwoVectors(src_vec, tree.model.ray);
    marker.pose.orientation.x = q_rot.x();
    marker.pose.orientation.y = q_rot.y();
    marker.pose.orientation.z = q_rot.z();
    marker.pose.orientation.w = q_rot.w();

    marker.scale.x = 2 * tree.model.radius;
    marker.scale.y = 2 * tree.model.radius;
    marker.scale.z = 10;

    if (cylinderId < 200000) {
      marker.color.a = 0.5;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    } else {
      marker.color.a = 0.5;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
    }

    tMarkerArray.markers.push_back(marker);
    cylinderId++;
  }

  // if we use ros::Time::now() instead of ros::Time(), the msg will
  // automatically be overwritten, then maybe this is no longer needed? See
  // vizCubeModels for example
  for (auto i = cylinderId; i < cylinderId + 50; ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "quadrotor/map";
    marker.header.stamp = ros::Time();
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::DELETE;
    tMarkerArray.markers.push_back(marker);
  }
}

void vizCubeModels(const std::vector<Cube> &cubeModels,
                   visualization_msgs::MarkerArray &tMarkerArray,
                   size_t &cubeId, const bool &is_global_map) {
  float scan_map_alpha = 0.5;
  float global_map_alpha = 0.7;
  float alpha;
  float red, blue, green;
  ros::Time stamp;
  // To differentiate from local map, global map will (1) be transparent, (2)
  // display permanently and (3) use different color
  if (is_global_map) {
    alpha = global_map_alpha;
    stamp = ros::Time::now();
    red = 0.0;
    blue = 1.0;
    green = 1.0;
  } else {
    alpha = scan_map_alpha;
    stamp = ros::Time::now();
    red = 0.5;
    blue = 0.5;
    green = 0.5;
  }

  for (const auto &cur_cube : cubeModels) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "quadrotor/map";
    marker.header.stamp = stamp;
    marker.id = cubeId;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // Center of cube
    marker.pose.position.x = cur_cube.model.pose.x();
    marker.pose.position.y = cur_cube.model.pose.y();
    marker.pose.position.z = cur_cube.model.pose.z();

    // Orientation of cube
    Vector3 src_vec(0, 0, 1);
    gtsam::Quaternion q_rot = cur_cube.model.pose.rotation().toQuaternion();
    marker.pose.orientation.x = q_rot.x();
    marker.pose.orientation.y = q_rot.y();
    marker.pose.orientation.z = q_rot.z();
    marker.pose.orientation.w = q_rot.w();

    marker.scale.x = cur_cube.model.scale(0);
    marker.scale.y = cur_cube.model.scale(1);
    marker.scale.z = cur_cube.model.scale(2);

    marker.color.a = alpha;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;

    tMarkerArray.markers.push_back(marker);
    cubeId++;
  }

  // HACK TO MAKE SURE WE ALWAYS DELETE THE PREVIOUS CYLINDERS
  for (auto i = cubeId; i < cubeId + 100; ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "quadrotor/map";
    marker.header.stamp = ros::Time();
    marker.id = i;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::DELETE;
    tMarkerArray.markers.push_back(marker);
  }
}

visualization_msgs::MarkerArray vizGroundModel(
    const std::vector<Plane> &gplanes, const std::string &frame_id, int idx) {
  visualization_msgs::MarkerArray groundModels;
  for (const auto &g : gplanes) {
    auto gmodel = vizGroundModel(g, frame_id, idx);
    groundModels.markers.push_back(gmodel);
    idx++;
  }

  // HACK TO MAKE SURE WE ALWAYS DELETE THE PREVIOUS GROUND MODELS
  for (auto i = idx; i < idx + 300; ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "quadrotor/map";
    marker.header.stamp = ros::Time();
    marker.id = i;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::DELETE;
    marker.ns = "plane";
    groundModels.markers.push_back(marker);
  }
  return groundModels;
}
visualization_msgs::Marker vizGroundModel(const Plane &gplane,
                                          const std::string &frame_id,
                                          const int idx) {
  visualization_msgs::Marker cube;
  cube.type = visualization_msgs::Marker::CUBE;
  cube.action = visualization_msgs::Marker::ADD;
  cube.id = idx;
  cube.ns = "plane";
  cube.scale.x = 2.5;
  cube.scale.y = 2.5;
  cube.scale.z = 0.1;
  if (idx < 200) {
    cube.color.r = 1.0;
    cube.color.g = 1.0;
    cube.color.b = 0.0;
    cube.color.a = 0.5;
  } else {
    cube.color.r = 1.0;
    cube.color.g = 1.0;
    cube.color.b = 1.0;
    cube.color.a = 0.5;
  }

  cube.header.frame_id = frame_id;
  cube.header.stamp = ros::Time();
  cube.pose.position.x = gplane.model.centroid(0);
  cube.pose.position.y = gplane.model.centroid(1);
  cube.pose.position.z = gplane.model.centroid(2);

  Vector3 src_vec(0, 0, 1);
  Quat q_rot = Quat::FromTwoVectors(src_vec, gplane.model.plane.segment(0, 3));
  cube.pose.orientation.x = q_rot.x();
  cube.pose.orientation.y = q_rot.y();
  cube.pose.orientation.z = q_rot.z();
  cube.pose.orientation.w = q_rot.w();
  return cube;
}

void landmarksToCloud(const std::vector<std::vector<TreeVertex>> &landmarks,
                      CloudT::Ptr &cloud) {
  size_t numPoints = 0;
  size_t color_id = 0;
  std::vector<float> color_values((int)landmarks.size());

  std::iota(std::begin(color_values), std::end(color_values), 1);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::shuffle(color_values.begin(), color_values.end(), gen);

  ROS_DEBUG_STREAM("Color values size: " << color_values.size());
  for (const auto &tree : landmarks) {
    if (tree[0].treeId == -1) continue;
    for (auto vtx : tree) {
      for (const auto &point : vtx.points) {
        PointT vp = point;
        vp.intensity = color_values[color_id];
        cloud->push_back(vp);
        numPoints++;
      }
    }
    color_id++;
  }
  cloud->width = numPoints;
  cloud->height = 1;
  cloud->is_dense = false;
}

cv::Mat DecodeImage(const sensor_msgs::ImageConstPtr &image_msg) {
  cv::Mat image;
  cv_bridge::CvImagePtr input_bridge;
  try {
    input_bridge = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
    image = input_bridge->image;
  } catch (cv_bridge::Exception &ex) {
    ROS_ERROR("Failed to convert depth image");
  }
  return image;
}

}  // namespace sloam