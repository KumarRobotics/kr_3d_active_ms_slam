#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/buffer_core.h>
#include <tf2_eigen/tf2_eigen.h>

// #include "sv_geom/transformation.hpp"
#include "definitions.hpp"

using geometry_msgs::TransformStamped;

/**
 * @brief LookupTransformWithDelay
 * @param buffer_core
 * @param target_frame
 * @param source_frame
 * @param stamp
 * @param tf_stamped
 * @param delay_ms
 * @return
 */
boost::optional<TransformStamped> LookupTransformWithDelayMs(
    const tf2::BufferCore &core, const std::string &target_frame,
    const std::string &source_frame, const ros::Time &stamp, int delay_ms = 0);

inline geometry_msgs::Quaternion ToRosQuat(const Quat &q) {
  return tf2::toMsg(q);
}

inline geometry_msgs::Quaternion ToRosQuat(const SO3 &R) {
  return tf2::toMsg(R.unit_quaternion());
}

geometry_msgs::Pose ToRosPose(const SE3 &T) {
  geometry_msgs::Pose pose;
  pose.position = tf2::toMsg(T.translation());
  pose.orientation = ToRosQuat(T.so3());
  return pose;
}

/// Conversion between Ros and Sophus
inline Vector3 ToVector3(const geometry_msgs::Point &point) {
  return Vector3(point.x, point.y, point.z);
}

inline Vector3 ToVector3(const geometry_msgs::Vector3 &point) {
  return Vector3(point.x, point.y, point.z);
}

inline Quat ToQuat(const geometry_msgs::Quaternion &quat) {
  return Quat(quat.w, quat.x, quat.y, quat.z);
}

inline SO3 ToSO3(const geometry_msgs::Quaternion &quat) {
  return SO3(ToQuat(quat));
}

SE3 ToSE3(const geometry_msgs::Transform &transform) {
  return {ToSO3(transform.rotation), ToVector3(transform.translation)};
}

SE3 ToSE3(const geometry_msgs::Pose &pose) {
  return {ToSO3(pose.orientation), ToVector3(pose.position)};
}

inline geometry_msgs::Transform ToRosTransform(const SE3 &T) {
  geometry_msgs::Transform transform;
  tf2::toMsg(T.translation(), transform.translation);
  transform.rotation = ToRosQuat(T.so3());
  return transform;
}

inline geometry_msgs::Transform ToRosTransform(
    const geometry_msgs::Pose &pose) {
  geometry_msgs::Transform transform;
  transform.translation.x = pose.position.x;
  transform.translation.y = pose.position.y;
  transform.translation.z = pose.position.z;
  transform.rotation = pose.orientation;
  return transform;
}
