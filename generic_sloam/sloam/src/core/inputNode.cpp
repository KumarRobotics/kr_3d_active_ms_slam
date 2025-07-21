#include <cube.h>
#include <definitions.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sloamNode.h>
#include <std_msgs/Header.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <deque>
#include <queue>

struct StampedSE3 {
  StampedSE3(SE3 p, ros::Time s) : pose(p), stamp(s){};
  StampedSE3() : pose(SE3()), stamp(ros::Time::now()){};
  SE3 pose;
  ros::Time stamp;
};

enum { CLOUD_TOO_OLD, CLOUD_TOO_NEW, CLOUD_FOUND };

class InputManager {
 public:
  explicit InputManager(ros::NodeHandle nh);
  bool RunInputNode(std::deque<StampedSE3> &odomQueue, const int &robotID);

  std::deque<StampedSE3> robot1OdomQueue_;
  std::deque<StampedSE3> robot2OdomQueue_;

 private:
  ros::Subscriber Robot1CubeSub_;
  ros::Subscriber Robot2CubeSub_;

  void updateLastPose(const StampedSE3 &odom, const int &robotID);

  void Robot1OdomCb_(const nav_msgs::OdometryConstPtr &odom_msg);
  void Robot2OdomCb_(const nav_msgs::OdometryConstPtr &odom_msg);

  int SyncSemantics(const ros::Time stamp, CloudT::Ptr treeCloud,
                    CloudT::Ptr groundCloud, std::vector<Cube> &cubesBody,
                    const int &robotID);
  // void PCCb_(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
  void Robot1TreePCCb_(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
  void Robot1GroundPCCb_(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
  void Robot1CubeCb_(const visualization_msgs::MarkerArray &cuboid_msg);
  void Robot2TreePCCb_(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
  void Robot2GroundPCCb_(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
  void Robot2CubeCb_(const visualization_msgs::MarkerArray &cuboid_msg);

  double max_timestamp_offset_ = 0.01;

  bool callSLOAM(SE3 relativeMotion, ros::Time stamp,
                 std::deque<StampedSE3> &odomQueue, const int &robotID);
  void PublishAccumOdom_(const SE3 &relativeMotion);
  void Odom2SlamTf();
  void PublishOdomAsTf(const nav_msgs::Odometry &odom_msg,
                       const std::string &parent_frame_id,
                       const std::string &child_frame_id);

  std::queue<sensor_msgs::PointCloud2ConstPtr> robot1TreePcQueue_;
  std::queue<sensor_msgs::PointCloud2ConstPtr> robot1GroundPcQueue_;
  std::queue<std::pair<std::vector<Cube>, ros::Time>> robot1CubesQueue_;

  std::queue<sensor_msgs::PointCloud2ConstPtr> robot2TreePcQueue_;
  std::queue<sensor_msgs::PointCloud2ConstPtr> robot2GroundPcQueue_;
  std::queue<std::pair<std::vector<Cube>, ros::Time>> robot2CubesQueue_;

  ros::NodeHandle nh_;
  ros::Publisher pubRobot1HighFreqSLOAMPose_;
  ros::Publisher pubRobot2HighFreqSLOAMPose_;
  ros::Publisher pubRobot1HighFreqSLOAMOdom_;
  ros::Publisher pubRobot2HighFreqSLOAMOdom_;
  ros::Subscriber Robot1OdomSub_;
  ros::Subscriber Robot2OdomSub_;

  ros::Subscriber Robot1GroundPCSub_;
  ros::Subscriber Robot1TreePCSub_;
  ros::Subscriber Robot2GroundPCSub_;
  ros::Subscriber Robot2TreePCSub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster broadcaster_;

  // params
  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string robot_ns_prefix_;
  int number_of_robots_;
  std::string odom_topic_;
  std::string robot_frame_id_;
  // std::string cloud_topic_;
  std::vector<std::string> robot_odom_topics_;
  std::string robot1_odom_topic_;
  std::string robot2_odom_topic_;

  float minOdomDistance_;
  float minSLOAMAltitude_;
  size_t maxQueueSize_;

  // flags
  bool robot1TreeCloudUpdated_ = false;
  bool robot1GroundCloudUpdated_ = false;
  bool robot1CubesUpdated_ = false;

  bool robot2TreeCloudUpdated_ = false;
  bool robot2GroundCloudUpdated_ = false;
  bool robot2CubesUpdated_ = false;

  bool robot1OdomReceived_ = false;
  bool robot2OdomReceived_ = false;
  // vars
  boost::shared_ptr<sloam::SLOAMNode> sloam_ = nullptr;
  std::vector<SE3> robot1KeyPoses_;
  std::vector<SE3> robot2KeyPoses_;

  // since we initialize the latestOdom to be identity, upon the first callback,
  // the relativeMotion will be the first pose (i.e. robot_to_world transform)
  StampedSE3 robot1LatestOdom;
  SE3 robot1LastSLOAMKeyPose_ = SE3();
  StampedSE3 robot2LatestOdom;
  SE3 robot2LastSLOAMKeyPose_ = SE3();

  bool robot1FirstOdom_;
  bool robot2FirstOdom_;
  bool publishTf_;
  size_t robot1OdomCounter_;
  size_t robot2OdomCounter_;

  size_t odomFreqFilter_;
};

InputManager::InputManager(ros::NodeHandle nh)
    : nh_(nh), tf_listener_{tf_buffer_} {
  nh_.param<float>("min_odom_distance", minOdomDistance_, 0.5);
  nh_.param<float>("min_sloam_z", minSLOAMAltitude_, 0.0);
  maxQueueSize_ = nh_.param("max_queue_size", 30);
  odomFreqFilter_ = nh_.param("odom_freq_filter", 20);
  publishTf_ = nh_.param("publish_tf", true);
  nh_.param<int>("number_of_robots", number_of_robots_, 1);
  nh_.param<std::string>("robot_ns_prefix", robot_ns_prefix_, "robot");
  nh_.param<std::string>("odom_topic", odom_topic_, "odom");
  std::string cur_robot_odom_topic;
  int robot_actual_ID;
  for (int robot_id = 0; robot_id < number_of_robots_; robot_id++) {
    // assign odom topics
    robot_actual_ID = robot_id + 1;
    cur_robot_odom_topic =
        robot_ns_prefix_ + std::to_string(robot_actual_ID) + "/" + odom_topic_;
    robot_odom_topics_.push_back(cur_robot_odom_topic);

    // temporarily only supporting 2 robots
    if (robot_actual_ID == 1) {
      robot1_odom_topic_ = cur_robot_odom_topic;
    } else if (robot_actual_ID == 2) {
      robot2_odom_topic_ = cur_robot_odom_topic;
    }
  }

  nh_.param<std::string>("robot_frame_id", robot_frame_id_, "robot");
  nh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
  nh_.param<std::string>("map_frame_id", map_frame_id_, "map");

  pubRobot1HighFreqSLOAMPose_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "quadrotor1/pose_high_freq", 10);
  pubRobot2HighFreqSLOAMPose_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "quadrotor2/pose_high_freq", 10);

  pubRobot1HighFreqSLOAMOdom_ =
      nh_.advertise<nav_msgs::Odometry>("quadrotor1/sloam_odom_high_freq", 20);
  pubRobot2HighFreqSLOAMOdom_ =
      nh_.advertise<nav_msgs::Odometry>("quadrotor2/sloam_odom_high_freq", 20);

  Robot1OdomSub_ =
      nh_.subscribe(robot1_odom_topic_, 10, &InputManager::Robot1OdomCb_, this);
  Robot2OdomSub_ =
      nh_.subscribe(robot2_odom_topic_, 10, &InputManager::Robot2OdomCb_, this);

  Robot1GroundPCSub_ = nh_.subscribe("/quadrotor1/ground_cloud", 10,
                                     &InputManager::Robot1GroundPCCb_, this);
  Robot1TreePCSub_ = nh_.subscribe("/quadrotor1/tree_cloud", 10,
                                   &InputManager::Robot1TreePCCb_, this);
  Robot1CubeSub_ = nh_.subscribe("/quadrotor1/car_cuboids_body", 10,
                                 &InputManager::Robot1CubeCb_, this);
  Robot2GroundPCSub_ = nh_.subscribe("/quadrotor2/ground_cloud", 10,
                                     &InputManager::Robot2GroundPCCb_, this);
  Robot2TreePCSub_ = nh_.subscribe("/quadrotor2/tree_cloud", 10,
                                   &InputManager::Robot2TreePCCb_, this);
  Robot2CubeSub_ = nh_.subscribe("/quadrotor2/car_cuboids_body", 10,
                                 &InputManager::Robot2CubeCb_, this);

  auto sloam_ptr = boost::make_shared<sloam::SLOAMNode>(nh_);
  sloam_ = std::move(sloam_ptr);
  robot1FirstOdom_ = true;
  robot2FirstOdom_ = true;

  robot1OdomCounter_ = 0;
  robot2OdomCounter_ = 0;

  ROS_INFO("SLOAM initialized");
}

void InputManager::Robot1OdomCb_(const nav_msgs::OdometryConstPtr &odom_msg) {
  if (odomFreqFilter_ > 1) {
    robot1OdomCounter_++;
    if (robot1OdomCounter_ % odomFreqFilter_ != 0) return;
    robot1OdomCounter_ = 0;
  }

  auto pose = odom_msg->pose.pose;
  ros::Time odomStamp = odom_msg->header.stamp;
  Quat rot(pose.orientation.w, pose.orientation.x, pose.orientation.y,
           pose.orientation.z);
  Vector3 pos(pose.position.x, pose.position.y, pose.position.z);

  SE3 odom = SE3();
  odom.setQuaternion(rot);
  odom.translation() = pos;

  // odom is in cam frame, we need it in robot frame
  geometry_msgs::TransformStamped transform_cam_body;
  try {
    auto transform_cam_body = tf_buffer_.lookupTransform(
        odom_msg->child_frame_id, robot_frame_id_, ros::Time(0));
    SE3 cam_body_tf(
        tf2::transformToEigen(transform_cam_body).matrix().cast<double>());
    odom = odom * cam_body_tf;
  } catch (tf2::TransformException &ex) {
    ROS_INFO_THROTTLE(10,
                      "Camera to body transform is not found, now setting it "
                      "as identity... If you're in simulator, ignore this.");
  }

  // robot has never been higher than height (Z) threshold
  if (robot1FirstOdom_ && pose.position.z < minSLOAMAltitude_) {
    ROS_INFO_STREAM_THROTTLE(
        5, "Quad is too low, will not call sloam, height threshold is "
               << minSLOAMAltitude_);
    return;
  }

  robot1OdomQueue_.emplace_back(odom, odomStamp);
  if (robot1OdomQueue_.size() > 10 * maxQueueSize_)
    robot1OdomQueue_.pop_front();
}

void InputManager::Robot2OdomCb_(const nav_msgs::OdometryConstPtr &odom_msg) {
  if (odomFreqFilter_ > 1) {
    robot2OdomCounter_++;
    if (robot2OdomCounter_ % odomFreqFilter_ != 0) return;
    robot2OdomCounter_ = 0;
  }

  auto pose = odom_msg->pose.pose;
  ros::Time odomStamp = odom_msg->header.stamp;
  Quat rot(pose.orientation.w, pose.orientation.x, pose.orientation.y,
           pose.orientation.z);
  Vector3 pos(pose.position.x, pose.position.y, pose.position.z);

  SE3 odom = SE3();
  odom.setQuaternion(rot);
  odom.translation() = pos;

  // odom is in cam frame, we need it in robot frame
  geometry_msgs::TransformStamped transform_cam_body;
  try {
    auto transform_cam_body = tf_buffer_.lookupTransform(
        odom_msg->child_frame_id, robot_frame_id_, ros::Time(0));
    SE3 cam_body_tf(
        tf2::transformToEigen(transform_cam_body).matrix().cast<double>());
    odom = odom * cam_body_tf;
  } catch (tf2::TransformException &ex) {
    ROS_INFO_THROTTLE(10,
                      "Camera to body transform is not found, now setting it "
                      "as identity... If you're in simulator, ignore this.");
  }

  if (robot2FirstOdom_ && pose.position.z < minSLOAMAltitude_) {
    ROS_INFO_STREAM_THROTTLE(
        5, "Quad is too low, will not call sloam, height threshold is "
               << minSLOAMAltitude_);
    return;
  }

  robot2OdomQueue_.emplace_back(odom, odomStamp);
  if (robot2OdomQueue_.size() > 10 * maxQueueSize_)
    robot2OdomQueue_.pop_front();
}

bool InputManager::RunInputNode(std::deque<StampedSE3> &odomQueue,
                                const int &robotID) {
  if (odomQueue.size() == 0) {
    ROS_INFO_STREAM(
        "Odom queue is empty... either no odometry or no object detection "
        "received");
  } else {
    // first, publish the odometry based on the last sloam call's optimized pose
    // and current relative transform estimated by raw odometry
    SE3 highFreqSLOAMPose;

    if (robotID == 0) {
      if (robot1OdomReceived_) {
        // if odom has been updated in sloam, use the relative odom pose and add
        // it to the previous sloam pose
        SE3 latestRelativeMotion =
            robot1LatestOdom.pose.inverse() * odomQueue.back().pose;
        highFreqSLOAMPose = robot1LastSLOAMKeyPose_ * latestRelativeMotion;
      } else {
        // directly take current odometry and publish
        highFreqSLOAMPose = odomQueue.back().pose;
      }
      auto odom_stamp = odomQueue.back().stamp;
      pubRobot1HighFreqSLOAMPose_.publish(
          sloam::makeROSPose(highFreqSLOAMPose, map_frame_id_, odom_stamp));
      pubRobot1HighFreqSLOAMOdom_.publish(
          sloam::toRosOdom_(highFreqSLOAMPose, map_frame_id_, odom_stamp));
    } else if (robotID == 1) {
      if (robot2OdomReceived_) {
        // if odom has been updated in sloam, use the relative odom pose and add
        // it to the previous sloam pose
        SE3 latestRelativeMotion =
            robot2LatestOdom.pose.inverse() * odomQueue.back().pose;
        highFreqSLOAMPose = robot2LastSLOAMKeyPose_ * latestRelativeMotion;
      } else {
        // directly take current odometry and publish
        highFreqSLOAMPose = odomQueue.back().pose;
      }
      auto odom_stamp = odomQueue.back().stamp;
      pubRobot2HighFreqSLOAMPose_.publish(
          sloam::makeROSPose(highFreqSLOAMPose, map_frame_id_, odom_stamp));
      pubRobot2HighFreqSLOAMOdom_.publish(
          sloam::toRosOdom_(highFreqSLOAMPose, map_frame_id_, odom_stamp));
    }
  }

  std::queue<sensor_msgs::PointCloud2ConstPtr> *treePcQueuePtr;
  std::queue<sensor_msgs::PointCloud2ConstPtr> *groundPcQueuePtr;
  std::queue<std::pair<std::vector<Cube>, ros::Time>> *cubesQueuePtr;

  bool *treeCloudUpdatedPtr;
  bool *groundCloudUpdatedPtr;
  bool *cubesUpdatedPtr;

  if (robotID == 0) {
    treePcQueuePtr = &robot1TreePcQueue_;
    groundPcQueuePtr = &robot1GroundPcQueue_;
    cubesQueuePtr = &robot1CubesQueue_;
    treeCloudUpdatedPtr = &robot1TreeCloudUpdated_;
    groundCloudUpdatedPtr = &robot1GroundCloudUpdated_;
    cubesUpdatedPtr = &robot1CubesUpdated_;
  } else if (robotID == 1) {
    treePcQueuePtr = &robot2TreePcQueue_;
    groundPcQueuePtr = &robot2GroundPcQueue_;
    cubesQueuePtr = &robot2CubesQueue_;
    treeCloudUpdatedPtr = &robot2TreeCloudUpdated_;
    groundCloudUpdatedPtr = &robot2GroundCloudUpdated_;
    cubesUpdatedPtr = &robot2CubesUpdated_;
  }

  if (treePcQueuePtr->empty() || groundPcQueuePtr->empty() ||
      cubesQueuePtr->empty())
    return false;


  if ((*treeCloudUpdatedPtr) & (*groundCloudUpdatedPtr) & (*cubesUpdatedPtr)) {
    // TODO: we need to consider the case where cubes are not guaranteed to be
    // slower than treeCloud or groundCloud

    // cubes are published at lowest rate, thus using it to find the odometry
    // at the corresponding stamp
    double cube_stamp =
        cubesQueuePtr->back().second.toSec();  // use .back to take newest cube
    for (auto i = 0; i < odomQueue.size(); ++i) {
      auto odom = odomQueue[i];
      // ROS_ERROR_STREAM_THROTTLE(1, "odom size: " << odomQueue.size());

      if (odom.stamp.toSec() < cube_stamp - max_timestamp_offset_) {
        ROS_DEBUG_THROTTLE(1, "odom MSG TOO OLD");
        continue;
      } else if (odom.stamp.toSec() > cube_stamp + max_timestamp_offset_) {
        ROS_DEBUG_THROTTLE(1, "odom MSG TOO NEW");
        continue;
      } else {
        ROS_DEBUG_STREAM_THROTTLE(
            1, "odom stamp matched with cuboid stamp, the stamp is: "
                   << (int)cube_stamp);


        bool isFirstOdom;
        SE3 currRelativeMotion;
        if (robotID == 0) {
          // Use odom to estimate motion since last key frame
          currRelativeMotion = robot1LatestOdom.pose.inverse() * odom.pose;
          isFirstOdom = robot1FirstOdom_;
        } else if (robotID == 1) {
          currRelativeMotion = robot2LatestOdom.pose.inverse() * odom.pose;
          isFirstOdom = robot2FirstOdom_;
        }

        if (isFirstOdom) {
          // since we initialize the latestOdom to be identity, upon the first
          // callback, the relativeMotion will be the first pose (i.e.
          // robot_to_world transform)
          ROS_INFO_THROTTLE(1.0, "first sloam call");

          if (callSLOAM(currRelativeMotion, odom.stamp, odomQueue, robotID)) {
            if (robotID == 0) {
              robot1FirstOdom_ = false;
            } else if (robotID == 1) {
              robot2FirstOdom_ = false;
            }
            updateLastPose(odom, robotID);

            ROS_WARN(
                "inputNode.cpp: check why not return true after callSLOAM "
                "succeeds");
            // return true;
          }
        } else {
          double accumMovement = currRelativeMotion.translation().norm();
          if (accumMovement > minOdomDistance_) {
            ROS_DEBUG_THROTTLE(1.0, "Distance %f", (accumMovement));
            if (callSLOAM(currRelativeMotion, odom.stamp, odomQueue, robotID)) {
              updateLastPose(odom, robotID);

              ROS_WARN(
                  "inputNode.cpp: check why not return true after callSLOAM "
                  "succeeds");
              // return true;
            }
          }
        }
      }
    }
  }

  return false;
}


void InputManager::updateLastPose(const StampedSE3 &odom, const int &robotID) {
  if (robotID == 0) {
    robot1OdomReceived_ = true;
    robot1LatestOdom.pose = odom.pose;
    robot1LatestOdom.stamp = odom.stamp;
    if (robot1KeyPoses_.size() == 0) {
      robot1LastSLOAMKeyPose_ = robot1LatestOdom.pose;
    } else {
      robot1LastSLOAMKeyPose_ = robot1KeyPoses_.back();
    }
  } else if (robotID == 1) {
    robot2OdomReceived_ = true;
    robot2LatestOdom.pose = odom.pose;
    robot2LatestOdom.stamp = odom.stamp;
    if (robot2KeyPoses_.size() == 0) {
      robot2LastSLOAMKeyPose_ = robot2LatestOdom.pose;
    } else {
      robot2LastSLOAMKeyPose_ = robot2KeyPoses_.back();
    }
  }
}

bool InputManager::callSLOAM(SE3 relativeMotion, ros::Time stamp,
                             std::deque<StampedSE3> &odomQueue,
                             const int &robotID) {
  CloudT::Ptr treeCloud(new CloudT);
  CloudT::Ptr groundCloud(new CloudT);
  std::vector<Cube> cubesBody;
  auto r = SyncSemantics(stamp, treeCloud, groundCloud, cubesBody, robotID);
  if (r == CLOUD_FOUND) {
    // remove oldest element from odom queue
    odomQueue.pop_front();
    SE3 keyPose = SE3();

    // If first odom, since we initialize the latestOdom to be identity, the
    // relativeMotion will be the first pose (i.e. robot_to_world transform),
    // and we can just assume the prevKeyPose pose to be identity
    SE3 prevKeyPose;
    if (robotID == 0) {
      prevKeyPose = robot1FirstOdom_
                        ? SE3()
                        : robot1KeyPoses_[robot1KeyPoses_.size() - 1];
    } else if (robotID == 1) {
      prevKeyPose = robot2FirstOdom_
                        ? SE3()
                        : robot2KeyPoses_[robot2KeyPoses_.size() - 1];
    }

    pcl_conversions::toPCL(stamp, treeCloud->header.stamp);
    bool success =
        sloam_->runSLOAMNode(relativeMotion, prevKeyPose, treeCloud,
                             groundCloud, cubesBody, stamp, keyPose, robotID);
    if (success) {
      if (robotID == 0) {
        robot1KeyPoses_.push_back(keyPose);
      } else if (robotID == 1) {
        robot2KeyPoses_.push_back(keyPose);
      }
      return true;
    }
  } else {
    // remove oldest element from odom queue
    if (r == CLOUD_TOO_NEW) odomQueue.pop_front();
    ROS_DEBUG_THROTTLE(1, "Corresponding point cloud not found. Skipping.");
  }
  return false;
}



void InputManager::Robot1TreePCCb_(
    const sensor_msgs::PointCloud2ConstPtr &cloudMsg) {
  robot1TreePcQueue_.push(cloudMsg);
  if (robot1TreePcQueue_.size() > maxQueueSize_) robot1TreePcQueue_.pop();
  robot1TreeCloudUpdated_ = true;
}

void InputManager::Robot1GroundPCCb_(
    const sensor_msgs::PointCloud2ConstPtr &cloudMsg) {
  robot1GroundPcQueue_.push(cloudMsg);
  if (robot1GroundPcQueue_.size() > maxQueueSize_) robot1GroundPcQueue_.pop();
  robot1GroundCloudUpdated_ = true;
}

void InputManager::Robot2TreePCCb_(
    const sensor_msgs::PointCloud2ConstPtr &cloudMsg) {
  robot2TreePcQueue_.push(cloudMsg);
  if (robot2TreePcQueue_.size() > maxQueueSize_) robot2TreePcQueue_.pop();
  robot2TreeCloudUpdated_ = true;
}

void InputManager::Robot2GroundPCCb_(
    const sensor_msgs::PointCloud2ConstPtr &cloudMsg) {
  robot2GroundPcQueue_.push(cloudMsg);
  if (robot2GroundPcQueue_.size() > maxQueueSize_) robot2GroundPcQueue_.pop();
  robot2GroundCloudUpdated_ = true;
}

int InputManager::SyncSemantics(const ros::Time stamp, CloudT::Ptr treeCloud,
                                CloudT::Ptr groundCloud,
                                std::vector<Cube> &cubesBody,
                                const int &robotID) {
  std::queue<sensor_msgs::PointCloud2ConstPtr> *treePcQueuePtr;
  std::queue<sensor_msgs::PointCloud2ConstPtr> *groundPcQueuePtr;
  std::queue<std::pair<std::vector<Cube>, ros::Time>> *cubesQueuePtr;
  bool *treeCloudUpdatedPtr;
  bool *groundCloudUpdatedPtr;
  bool *cubesUpdatedPtr;

  if (robotID == 0) {
    treePcQueuePtr = &robot1TreePcQueue_;
    groundPcQueuePtr = &robot1GroundPcQueue_;
    cubesQueuePtr = &robot1CubesQueue_;
    treeCloudUpdatedPtr = &robot1TreeCloudUpdated_;
    groundCloudUpdatedPtr = &robot1GroundCloudUpdated_;
    cubesUpdatedPtr = &robot1CubesUpdated_;
  } else if (robotID == 1) {
    treePcQueuePtr = &robot2TreePcQueue_;
    groundPcQueuePtr = &robot2GroundPcQueue_;
    cubesQueuePtr = &robot2CubesQueue_;
    treeCloudUpdatedPtr = &robot2TreeCloudUpdated_;
    groundCloudUpdatedPtr = &robot2GroundCloudUpdated_;
    cubesUpdatedPtr = &robot2CubesUpdated_;
  }

  if (treePcQueuePtr->empty() || groundPcQueuePtr->empty() ||
      cubesQueuePtr->empty())
    return false;

  bool tree_cloud_found = false;
  bool ground_cloud_found = false;
  bool cubes_found = false;

  // total time for semantic segmentation and cuboid modeling to finish
  float wait_duration = 0.5;
  float check_interval = 0.05;
  int counter = 0;
  int total_num_loops = static_cast<int>(wait_duration / check_interval);

  while (++counter < total_num_loops) {
    // check the queue, remove the old msgs, only keep the ones after stamp
    if (*treeCloudUpdatedPtr & !tree_cloud_found) {
      while (!treePcQueuePtr->empty()) {
        if (treePcQueuePtr->front()->header.stamp.toSec() <
            stamp.toSec() - max_timestamp_offset_) {
          // message too old
          ROS_DEBUG_THROTTLE(1, "tree PC MSG TOO OLD");
          treePcQueuePtr->pop();
        } else if (treePcQueuePtr->front()->header.stamp.toSec() >
                   stamp.toSec() + max_timestamp_offset_) {
          // if the oldest cloud is too new, we just return since there will not
          // be older cloud coming in
          ROS_ERROR_STREAM(
              "Stamp diff: " << treePcQueuePtr->front()->header.stamp.toSec() -
                                    stamp.toSec());
          ROS_DEBUG_THROTTLE(1, "tree PC MSG TOO NEW");
          return CLOUD_TOO_NEW;
        } else {
          pcl::fromROSMsg(*(treePcQueuePtr->front()), *treeCloud);
          treePcQueuePtr->pop();
          tree_cloud_found = true;
          ROS_DEBUG("tree cloud found");
          break;
        }
      }
      *treeCloudUpdatedPtr = false;
    }

    // check the queue, remove the old msgs, only keep the ones after stamp
    if (*groundCloudUpdatedPtr & !ground_cloud_found) {
      while (!groundPcQueuePtr->empty()) {
        if (groundPcQueuePtr->front()->header.stamp.toSec() <
            stamp.toSec() - max_timestamp_offset_) {
          // message too old
          ROS_DEBUG_THROTTLE(1, "ground PC MSG TOO OLD");
          groundPcQueuePtr->pop();
        } else if (groundPcQueuePtr->front()->header.stamp.toSec() >
                   stamp.toSec() + max_timestamp_offset_) {
          ROS_DEBUG_THROTTLE(1, "ground PC MSG TOO NEW");
          // if the oldest cloud is too new, we just return since there will not
          // be older cloud coming in
          return CLOUD_TOO_NEW;
        } else {
          pcl::fromROSMsg(*groundPcQueuePtr->front(), *groundCloud);
          groundPcQueuePtr->pop();
          ground_cloud_found = true;
          ROS_DEBUG("ground cloud found");
          break;
        }
      }
      *groundCloudUpdatedPtr = false;
    }

    // check the queue, remove the old msgs, only keep the ones after stamp
    if (*cubesUpdatedPtr & !cubes_found) {
      while (!cubesQueuePtr->empty()) {
        double cube_stamp = cubesQueuePtr->front().second.toSec();
        if (cube_stamp < stamp.toSec() - max_timestamp_offset_) {
          // message too old
          ROS_DEBUG_THROTTLE(1, "cuboids MSG TOO OLD");
          cubesQueuePtr->pop();
        } else if (cube_stamp > stamp.toSec() + max_timestamp_offset_) {
          ROS_DEBUG_THROTTLE(1, "cuboids MSG TOO NEW");
          // if the oldest cloud is too new, we just return since there will not
          // be older cloud coming in
          return CLOUD_TOO_NEW;
        } else {
          cubesBody = cubesQueuePtr->front().first;
          cubesQueuePtr->pop();
          cubes_found = true;
          ROS_DEBUG("cuboids found");
          break;
        }
      }
      *cubesUpdatedPtr = false;
    }

    if (tree_cloud_found & ground_cloud_found & cubes_found) {
      ROS_DEBUG("Calling SLOAM");
      return CLOUD_FOUND;
    }
    ros::Duration(check_interval).sleep();
  }

  ROS_WARN_STREAM("Sync is not found after " << counter * check_interval
                                             << " seconds!");

  // no valid sync is found
  return CLOUD_TOO_OLD;
}


void InputManager::PublishOdomAsTf(const nav_msgs::Odometry &odom_msg,
                                   const std::string &parent_frame_id,
                                   const std::string &child_frame_id) {
  geometry_msgs::TransformStamped tf;
  tf.header = odom_msg.header;
  // note that normally parent_frame_id should be the same as
  // odom_msg.header.frame_id
  tf.header.frame_id = parent_frame_id;
  tf.child_frame_id = child_frame_id;
  tf.transform.translation.x = odom_msg.pose.pose.position.x;
  tf.transform.translation.y = odom_msg.pose.pose.position.y;
  tf.transform.translation.z = odom_msg.pose.pose.position.z;
  tf.transform.rotation = odom_msg.pose.pose.orientation;
  broadcaster_.sendTransform(tf);
}

void InputManager::Robot1CubeCb_(
    const visualization_msgs::MarkerArray &cuboid_msg) {
  std::vector<Cube> scan_cubes_body;
  int total_cuboids = cuboid_msg.markers.size();
  int cuboid_id;
  ros::Time cube_stamp;

  for (const visualization_msgs::Marker &cur_cuboid_marker :
       cuboid_msg.markers) {
    cuboid_id = cur_cuboid_marker.id;
    if (cuboid_id < (total_cuboids - 10)) {
      // cuboid is too old, discard
      continue;
    }
    cube_stamp = cur_cuboid_marker.header.stamp;

    double x = cur_cuboid_marker.pose.position.x;
    double y = cur_cuboid_marker.pose.position.y;
    double z = cur_cuboid_marker.pose.position.z;

    // using constructor Rot3 (double w, double x, double y, double z)
    gtsam::Rot3 rot(cur_cuboid_marker.pose.orientation.w,
                    cur_cuboid_marker.pose.orientation.x,
                    cur_cuboid_marker.pose.orientation.y,
                    cur_cuboid_marker.pose.orientation.z);

    gtsam::Point3 trans(x, y, z);

    // pose of the cuboid
    gtsam::Pose3 pose(rot, trans);
    // scale of the cuboid
    const gtsam::Point3 scale(cur_cuboid_marker.scale.x,
                              cur_cuboid_marker.scale.y,
                              cur_cuboid_marker.scale.z);
    // add cuboid to scan_cubes_body_
    scan_cubes_body.push_back(Cube(pose, scale));
  }

  if (robot1CubesQueue_.size() > maxQueueSize_) robot1CubesQueue_.pop();

  auto cur_cube_data = std::make_pair(scan_cubes_body, cube_stamp);
  robot1CubesQueue_.push(cur_cube_data);

  robot1CubesUpdated_ = true;
}

void InputManager::Robot2CubeCb_(
    const visualization_msgs::MarkerArray &cuboid_msg) {
  std::vector<Cube> scan_cubes_body;
  int total_cuboids = cuboid_msg.markers.size();
  int cuboid_id;
  ros::Time cube_stamp;

  for (const visualization_msgs::Marker &cur_cuboid_marker :
       cuboid_msg.markers) {
    cuboid_id = cur_cuboid_marker.id;
    if (cuboid_id < (total_cuboids - 10)) {
      // cuboid is too old, discard
      continue;
    }
    cube_stamp = cur_cuboid_marker.header.stamp;

    double x = cur_cuboid_marker.pose.position.x;
    double y = cur_cuboid_marker.pose.position.y;
    double z = cur_cuboid_marker.pose.position.z;

    // using constructor Rot3 (double w, double x, double y, double z)
    gtsam::Rot3 rot(cur_cuboid_marker.pose.orientation.w,
                    cur_cuboid_marker.pose.orientation.x,
                    cur_cuboid_marker.pose.orientation.y,
                    cur_cuboid_marker.pose.orientation.z);

    gtsam::Point3 trans(x, y, z);

    // pose of the cuboid
    gtsam::Pose3 pose(rot, trans);
    // scale of the cuboid
    const gtsam::Point3 scale(cur_cuboid_marker.scale.x,
                              cur_cuboid_marker.scale.y,
                              cur_cuboid_marker.scale.z);
    // add cuboid to scan_cubes_body_
    scan_cubes_body.push_back(Cube(pose, scale));
  }

  if (robot2CubesQueue_.size() > maxQueueSize_) robot2CubesQueue_.pop();

  auto cur_cube_data = std::make_pair(scan_cubes_body, cube_stamp);
  robot2CubesQueue_.push(cur_cube_data);
  robot2CubesUpdated_ = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sloam");
  ros::NodeHandle n("sloam");
  InputManager in(n);

  ros::Rate r(30);  // 10 hz
  while (ros::ok()) {
    for (auto i = 0; i < 10; ++i) {
      ros::spinOnce();
      for (int robotID = 0; robotID < 2; ++robotID) {
        if (robotID == 0) {
          in.RunInputNode(in.robot1OdomQueue_, robotID);
        } else if (robotID == 1) {
          in.RunInputNode(in.robot2OdomQueue_, robotID);
        }
      }
      r.sleep();
    }
  }

  return 0;
}