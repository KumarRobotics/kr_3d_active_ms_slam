#include <cube.h>

Cube::Cube(const gtsam::Pose3 &pose, const gtsam::Point3 &scale) {
  model.pose = pose;
  // scale will contain x (length), y (width), z(height)
  model.scale = scale;
}

Scalar Cube::distance(const CubeParameters &input) const {
  return (input.pose.translation() - model.pose.translation()).norm();
};

Scalar Cube::distance(const PointT &point) const {
  gtsam::Point3 point_vec = gtsam::Point3(point.x, point.y, point.z);
  return (model.pose.translation() - point_vec).norm();
}

void Cube::project(const SE3 &tf) {
  // tf means tf_sensor_to_map, not tf_map_to_sensor
  gtsam::Pose3 pose_new = gtsam::Pose3(tf.matrix());
  model.pose = pose_new * model.pose;
  model.scale = model.scale;
}