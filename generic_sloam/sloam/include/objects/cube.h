#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <semanticObject.h>
#include <utils.h>

#include <algorithm>

// Cube object stores information about cuboid landmarks in the semantic map
struct CubeParameters {
  gtsam::Pose3 pose;
  gtsam::Point3 scale;
};

class Cube : public SemanticObject<CubeParameters> {
 public:
  explicit Cube(const gtsam::Pose3& pose, const gtsam::Point3& scale);

  Scalar distance(const CubeParameters& model) const;
  Scalar distance(const PointT& point) const;
  // tf means tf_sensor_to_map, not tf_map_to_sensor
  void project(const SE3& tf);
};
