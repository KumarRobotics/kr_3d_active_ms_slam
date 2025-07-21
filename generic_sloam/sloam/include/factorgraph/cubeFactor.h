#pragma once

#include <cube.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <Eigen/Dense>

namespace gtsam_cube {

// CubeMeasurement object is a warpper for the Cube object to work with GTSAM
class CubeMeasurement {
 public:
  CubeMeasurement(const Cube &c) {
    pose = c.model.pose;
    scale = c.model.scale;
  }

  CubeMeasurement(const gtsam::Pose3 &pose, const gtsam::Point3 &scale)
      : pose(pose), scale(scale){};

  gtsam::Pose3 pose;
  gtsam::Point3 scale;

  /**
   * Calculates the distance in tanget space between two cubes on the
   manifold
   * @param q cube landmark
   * @return vector from cube measurement to cube landmark in tangent space,
   * // q - m, instead of m - q, reference:
   * // https://gtsam.org/doxygen/a03188.html#a228acdaddcd94f5fec9a0d87350fe575
   */
  gtsam::Vector9 localCoordinates(const CubeMeasurement &q) const {
    gtsam::Vector9 v = gtsam::Vector9::Zero();
    // calculate pose error in SE3

    // Local coordinates of SE3 computation, check here for reference:
    // https://github.com/devbharat/gtsam/blob/5f15d264ed639cb0add335e2b089086141127fff/gtsam/geometry/Pose3.cpp~#L199
    // the following 3 options should do the same thing:

    // option 1: gtsam logmap
    // according to reference it should be q - m, i.e., pose.inverse()
    // * q.pose; why here is m - q?
    gtsam::Pose3 error_se3 = q.pose.inverse() * pose;
    gtsam::Vector6 error_se3_log = gtsam::Pose3::Logmap(error_se3);

    // follow this for analytic Jacobian calculation:
    // https://github.com/devbharat/gtsam/blob/5f15d264ed639cb0add335e2b089086141127fff/gtsam/geometry/Pose3.cpp~#L292
    // Pose3 Pose3::between(const Pose3 &p2, boost::optional<Matrix &> H1,
    //                      boost::optional<Matrix &> H2) const {
    //   Pose3 result = inverse() * p2;
    //   if (H1) *H1 = -result.inverse().AdjointMap();
    //   if (H2) *H2 = I6;
    //   return result;
    // }

    // option 2: gtsam localCoordinates
    // gtsam::Vector6 error_se3_log = pose.localCoordinates(q.pose);

    // option 3: gtsam localCoordinates (EXPMAP option -- using logmap and
    // between) between = compose(p2,inverse(p1));
    // gtsam::Vector6 error_se3_log =
    // gtsam::Pose3::Logmap(pose.between(q.pose));

    // Use sophus:
    // SE3 pose1 = SE3(pose.matrix());
    // SE3 pose2 = SE3(q.pose.matrix());
    // SE3 error_se3 = pose1 * pose2.inverse();
    // gtsam::Vector6 error_log_reversed = gtsam::Vector6(error_se3.log());
    // gtsam::Vector6 error_se3_log;
    // error_se3_log.head(3) = error_log_reversed.tail(3);
    // error_se3_log.tail(3) = error_log_reversed.head(3);

    v.segment(0, 6) = error_se3_log;
    // according to reference it should be q - m, i.e., q.scale -
    // scale; why here is m - q?
    v.segment(6, 3) = scale - q.scale;
    return v;
  }

  /**
   * Moves from this by v in tangent space, then retracts back to a
   * CubeMeasurement
   * @param v displacement vector in tangent space
   * @return CubeMeasurement on the manifold
   */
  CubeMeasurement retract(const gtsam::Vector9 &v) const {
    // Option 1: use Cayley map (defult in GTSAM)
    gtsam::Pose3 pose_new = pose.retract(v.segment(0, 6));

    // Option 2: use EXP map (requires GTSAM > 4.1, should be more accurate)
    // gtsam::Pose3 pose_new = pose.retract(v.segment(0, 6),
    // gtsam::Pose3::EXPMAP);

    // Option 3: Sophus SE3 exponential map for retraction, slower but seems
    // more accurate

    // gtsam::Vector6 v_input;
    // v_input.head(3) = v.segment(3, 3);
    // v_input.tail(3) = v.head(3);
    // SE3 pose_sophus = SE3::exp(v_input);
    // gtsam::Pose3 pose = gtsam::Pose3(pose_sophus.matrix());
    // SE3 pose_new_sophus = SE3(pose.matrix()) * pose_new_sophus;
    // gtsam::Pose3 pose_new = gtsam::Pose3(pose_new_sophus.matrix());

    // convert to gtsam point so that we can use retract
    gtsam::Point3 scale_new = scale.retract(v.segment(6, 3));
    return CubeMeasurement(pose_new, scale_new);
  }

  /**
   * Obtain the projection of the cylinder
   * @param p pose, p is tf_sensor_to_map, not tf_map_to_sensor
   * @return Cube position, axis and radius
   */
  CubeMeasurement project(const gtsam::Pose3 &p) const {
    // projecting from sensor frame to map frame
    // p is tf_sensor_to_map, not tf_map_to_sensor
    gtsam::Pose3 pose_new = p * pose;
    gtsam::Point3 scale_new = scale;
    return CubeMeasurement(pose_new, scale_new);
  }

  /* *************************************************************************
   */
  static CubeMeasurement Retract(const gtsam::Vector9 &v) {
    gtsam::Vector6 pose_vec = v.segment(0, 6);
    gtsam::Pose3 pose = gtsam::Pose3::Expmap(pose_vec);

    // Use sophus:
    // gtsam::Vector6 v_input;
    // v_input.head(3) = v.segment(3, 3);
    // v_input.tail(3) = v.head(3);
    // SE3 pose_new_sophus = SE3::exp(v_input);
    // gtsam::Pose3 pose = gtsam::Pose3(pose_new_sophus.matrix());

    gtsam::Point3 scale = v.segment(6, 3);
    return CubeMeasurement(pose, scale);
  }

  static gtsam::Vector9 LocalCoordinates(const CubeMeasurement &q) {
    gtsam::Vector9 v = gtsam::Vector9::Zero();
    v.segment(0, 6) = gtsam::Vector6(gtsam::Pose3::Logmap(q.pose));

    // Use sophus:
    // SE3 pose1 = SE3(q.pose.matrix());
    // gtsam::Vector6 pose_log_reversed = gtsam::Vector6(pose1.log());
    // gtsam::Vector6 pose_se3_log;
    // pose_se3_log.head(3) = pose_log_reversed.tail(3);
    // pose_se3_log.tail(3) = pose_log_reversed.head(3);
    // v.segment(0, 6) = pose_se3_log;

    v.segment(6, 3) = gtsam::Point3(q.scale);
    return v;
  }

  void print(const std::string &s = "") const {
    cout << s;
  }

  bool equals(const CubeMeasurement &other, double tol = 1e-9) const {
    return pose.equals(other.pose, tol) && scale.equals(other.scale, tol);
  }

  enum { dimension = 9 };
};

// CubeFactor is a class derived from the NoiseModelFactor2 class in GTSAM,
// the latter is a convenient base class for creating custom NoiseModelFactor
// with 2 variables. We need to implement evaluateError()
class CubeFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, CubeMeasurement> {
 protected:
  typedef NoiseModelFactor2<gtsam::Pose3, CubeMeasurement> Base;
  CubeMeasurement m_;

 public:
  CubeFactor(const gtsam::Key &poseKey, const gtsam::Key &cubeKey,
             const CubeMeasurement &q, const gtsam::SharedNoiseModel &model)
      : Base(model, poseKey, cubeKey), m_(q){};

  /**
   * Evaluate the error based on pose and measurements
   * https://gtsam.org/doxygen/a04136.html#ae97cf77232a179037cc8076aa4c6a591
   * @param q cube landmark, is related to h(x), z is related to m_
   * @return the factor's error h(x) - z
   */

  gtsam::Vector evaluateError(
      const gtsam::Pose3 &p, const CubeMeasurement &q,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none) const;

  CubeMeasurement measurement() const { return m_; }

  /** Returns the pose key */
  gtsam::Key poseKey() const { return key1(); }

  /** Returns the object/landmark key */
  gtsam::Key objectKey() const { return key2(); }
};

}  // namespace gtsam_cube

template <>
struct gtsam::traits<gtsam_cube::CubeMeasurement>
    : public gtsam::internal::Manifold<gtsam_cube::CubeMeasurement> {};

template <>
struct gtsam::traits<const gtsam_cube::CubeMeasurement>
    : public gtsam::internal::Manifold<gtsam_cube::CubeMeasurement> {};
