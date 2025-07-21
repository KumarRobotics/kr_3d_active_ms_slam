#pragma once

#include <cylinder.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <Eigen/Dense>

namespace gtsam_cylinder {
// CylinderMeasurement object is a warpper for the Cylinder object to work with
// GTSAM
class CylinderMeasurement {
 public:
  CylinderMeasurement(const Cylinder &c) {
    radius = c.model.radius;
    root = c.model.root;
    ray = c.model.ray;
  }

  CylinderMeasurement(const gtsam::Point3 &pos, const gtsam::Point3 &axis,
                      const double r)
      : root(pos), ray(axis), radius(r){};

  double radius;
  gtsam::Point3 root;
  gtsam::Point3 ray;
  /**
   * Calculates the distance in tanget space between two cylinders on the
   * manifold
   * @param q another CylinderMeasurement
   * @return vector between cylinders in tangent space, q - m(i.e. radius, root,
   * ray), instead of m - q, reference:
   * https://gtsam.org/doxygen/a03188.html#a228acdaddcd94f5fec9a0d87350fe575
   */
  gtsam::Vector7 localCoordinates(const CylinderMeasurement &q) const {
    gtsam::Vector7 v = gtsam::Vector7::Zero();
    v.segment(0, 3) = ray.localCoordinates(q.ray);
    v.segment(3, 3) = root.localCoordinates(q.root);
    // Remove absolute since it should just be h(x) - z
    v[6] = radius - q.radius;
    return v;
  }

  /**
   * Moves from this by v in tangent space, then retracts back to a
   * CylinderMeasurement
   * @param v displacement vector in tangent space
   * @return CylinderMeasurement on the manifold
   */
  CylinderMeasurement retract(const gtsam::Vector7 &v) const {
    gtsam::Point3 axis = ray.retract(v.segment(0, 3));
    gtsam::Point3 pos = root.retract(v.segment(3, 3));
    double r = radius + v.tail<1>()[0];
    return CylinderMeasurement(pos, axis, r);
  }

  /**
   * Obtain the projection of the cylinder
   * @param p pose
   * @return Cylinder position, axis and radius
   */
  CylinderMeasurement project(const gtsam::Pose3 &p) const {
    // p means tf_sensor_to_map, not tf_map_to_sensor
    // projecting from sensor frame to map frame
    gtsam::Point3 pos = p * root;
    gtsam::Point3 axis = p.rotation() * ray;
    return CylinderMeasurement(pos, axis, radius);
  }

  /* *************************************************************************
   */
  static CylinderMeasurement Retract(const gtsam::Vector7 &v) {
    gtsam::Point3 axis = gtsam::Point3(v.segment(0, 3));
    gtsam::Point3 pos = gtsam::Point3(v.segment(3, 3));
    double r = v.tail<1>()[0];
    return CylinderMeasurement(pos, axis, r);
  }

  static gtsam::Vector7 LocalCoordinates(const CylinderMeasurement &q) {
    gtsam::Vector7 v = gtsam::Vector7::Zero();
    v.segment(0, 3) = gtsam::Vector3(q.ray.x(), q.ray.y(), q.ray.z());
    v.segment(3, 3) = gtsam::Vector3(q.root.x(), q.root.y(), q.root.z());
    v[6] = q.radius;
    return v;
  }

  void print(const std::string &s = "") const {
    cout << s;
    cout << "Root\n" << root << endl;
    cout << "Ray\n" << ray << endl;
    cout << "Radius" << radius << endl;
  }

  bool equals(const CylinderMeasurement &other, double tol = 1e-9) const {
    return root.equals(other.root, tol) && ray.equals(other.ray, tol) &&
           abs(radius - other.radius) < tol;
  }

  enum { dimension = 7 };
};

class CylinderFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, CylinderMeasurement> {
 protected:
  typedef NoiseModelFactor2<gtsam::Pose3, CylinderMeasurement>
      Base;  ///< base class has keys and noisemodel as private members
  CylinderMeasurement m_;

 public:
  CylinderFactor(const gtsam::Key &poseKey, const gtsam::Key &cylKey,
                 const CylinderMeasurement &q,
                 const gtsam::SharedNoiseModel &model)
      : Base(model, poseKey, cylKey), m_(q){};

  /**  Evaluate the error */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &p, const CylinderMeasurement &q,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none) const;

  CylinderMeasurement measurement() const { return m_; }

  /** Returns the pose key */
  gtsam::Key poseKey() const { return key1(); }

  /** Returns the object/landmark key */
  gtsam::Key objectKey() const { return key2(); }

  gtsam::Matrix evaluateH1(const gtsam::Pose3 &p,
                           const CylinderMeasurement &cylinder) const;

  gtsam::Matrix evaluateH2(const gtsam::Pose3 &p,
                           const CylinderMeasurement &cylinder) const;

  gtsam::Matrix evaluateH1(const gtsam::Values &x) const;

  gtsam::Matrix evaluateH2(const gtsam::Values &x) const;

  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override;

  /** Returns true if equal keys, measurement, noisemodel and calibration */
  bool equals(const CylinderFactor &other, double tol = 1e-9) const;
};

}  // namespace gtsam_cylinder

template <>
struct gtsam::traits<gtsam_cylinder::CylinderMeasurement>
    : public gtsam::internal::Manifold<gtsam_cylinder::CylinderMeasurement> {};

template <>
struct gtsam::traits<const gtsam_cylinder::CylinderMeasurement>
    : public gtsam::internal::Manifold<gtsam_cylinder::CylinderMeasurement> {};
