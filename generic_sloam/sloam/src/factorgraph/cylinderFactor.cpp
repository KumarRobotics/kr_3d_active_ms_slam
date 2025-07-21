#include <cylinderFactor.h>
#include <gtsam/base/numericalDerivative.h>

using namespace std;

namespace gtsam_cylinder {

/* ************************************************************************* */
// this is called internalled by GTSAM, this q is what we are trying to solve, q
// is the object measurements in map frame
gtsam::Vector CylinderFactor::evaluateError(
    const gtsam::Pose3 &p, const CylinderMeasurement &q,
    boost::optional<gtsam::Matrix &> H1,
    boost::optional<gtsam::Matrix &> H2) const {
  // evaluate error

  // check what will happen if we express error in sensor frame

  // (1) our measurement error has to be expressed in the map frame to
  // get it working properly, and
  // (2)the error is h(x) - z, i.e.,
  // the error = cylinder^map_landmark - cylinder^map_measure

  // p is tf_sensor_to_map, not tf_map_to_sensor

  // m_ is the cylinder measurement  in the sensor frame,
  // cylinder^sensor_measure

  // q is the cylinder landmark in the map frame
  gtsam::Vector7 error = m_.project(p).localCoordinates(q);
  boost::function<gtsam::Vector(const gtsam::Pose3 &,
                                const CylinderMeasurement &)>
      funPtr(boost::bind(&CylinderFactor::evaluateError, this, _1, _2,
                         boost::none, boost::none));
  if (H1) {
    Eigen::Matrix<double, 7, 6> de_dx =
        gtsam::numericalDerivative21(funPtr, p, q, 1e-6);
    *H1 = de_dx;
  }
  if (H2) {
    Eigen::Matrix<double, 7, 7> de_dc =
        gtsam::numericalDerivative22(funPtr, p, q, 1e-6);
    *H2 = de_dc;
  }
  return error;
}

/* ************************************************************************* */
gtsam::Matrix CylinderFactor::evaluateH1(const gtsam::Pose3 &p,
                                         const CylinderMeasurement &cyl) const {
  gtsam::Matrix H1;
  this->evaluateError(p, cyl, H1, boost::none);
  return H1;
}

/* ************************************************************************* */
gtsam::Matrix CylinderFactor::evaluateH2(const gtsam::Pose3 &p,
                                         const CylinderMeasurement &cyl) const {
  gtsam::Matrix H2;
  this->evaluateError(p, cyl, boost::none, H2);
  return H2;
}

gtsam::Matrix CylinderFactor::evaluateH1(const gtsam::Values &x) const {
  const gtsam::Pose3 p = x.at<gtsam::Pose3>(this->poseKey());
  const CylinderMeasurement cyl = x.at<CylinderMeasurement>(this->objectKey());
  gtsam::Matrix H1;
  this->evaluateError(p, cyl, H1, boost::none);
  return H1;
}

/* ************************************************************************* */
gtsam::Matrix CylinderFactor::evaluateH2(const gtsam::Values &x) const {
  const gtsam::Pose3 p = x.at<gtsam::Pose3>(this->poseKey());
  const CylinderMeasurement cyl = x.at<CylinderMeasurement>(this->objectKey());
  gtsam::Matrix H2;
  this->evaluateError(p, cyl, boost::none, H2);
  return H2;
}

/* ************************************************************************* */
void CylinderFactor::print(const std::string &s,
                           const gtsam::KeyFormatter &keyFormatter) const {
  cout << s << "CylinderFactor(" << keyFormatter(key1()) << " -> "
       << keyFormatter(key2()) << ")" << endl;
  cout << "    NoiseModel: ";
  noiseModel()->print();
  cout << endl;
}

/* ************************************************************************* */
bool CylinderFactor::equals(const CylinderFactor &other, double tol) const {
  bool equal = noiseModel()->equals(*other.noiseModel(), tol) &&
               key1() == other.key1() && key2() == key2();
  return equal;
}

}  // namespace gtsam_cylinder