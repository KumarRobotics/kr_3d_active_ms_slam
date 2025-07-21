#include <cubeFactor.h>
#include <gtsam/base/numericalDerivative.h>

using namespace std;

namespace gtsam_cube {

gtsam::Vector CubeFactor::evaluateError(
    const gtsam::Pose3 &p, const CubeMeasurement &cube_lmrk,
    boost::optional<gtsam::Matrix &> H1,
    boost::optional<gtsam::Matrix &> H2) const {


  // (1) our measurement error has to be expressed in the map frame to
  // get it working properly, and
  // (2)the error is h(x) - z, i.e.,
  // the error = cube^map_landmark - cube^map_measure, i.e., log map of:
  // cube^map_measure.inverse()*(tf_sensor_to_map*cube^sensor_landmark)

  // p is tf_sensor_to_map, not tf_map_to_sensor
  // m_ is the cube measurement in the sensor frame, cube^sensor_measure
  // cube_lmrk is the cube landmark in the map frame, cube^map_landmark
  // The first 6 term in our error is the SE3 error
  gtsam::Vector9 error = m_.project(p).localCoordinates(cube_lmrk);

  boost::function<gtsam::Vector(const gtsam::Pose3 &, const CubeMeasurement &)>
      funPtr(boost::bind(&CubeFactor::evaluateError, this, _1, _2, boost::none,
                         boost::none));
  // Jacobian of error wrt pose
  if (H1) {
    Eigen::Matrix<double, 9, 6> de_dx =
        gtsam::numericalDerivative21(funPtr, p, cube_lmrk, 1e-6);
    *H1 = de_dx;
  }
  // Jacobian of error wrt cube measurement
  if (H2) {
    Eigen::Matrix<double, 9, 9> de_dc =
        gtsam::numericalDerivative22(funPtr, p, cube_lmrk, 1e-6);
    *H2 = de_dc;
  }
  return error;
}
}  // namespace gtsam_cube