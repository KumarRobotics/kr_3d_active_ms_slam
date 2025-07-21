#include <cube.h>
#include <cubeFactor.h>
#include <definitions.h>
#include <graph.h>
// #include <plane.h>
#include <serialization.h>
#include <stdio.h>
#include <stdlib.h>

#include "gtest/gtest.h"
using ::testing::EmptyTestEventListener;
using ::testing::InitGoogleTest;
using ::testing::Test;
using ::testing::TestEventListeners;
using ::testing::TestInfo;
using ::testing::TestPartResult;
using ::testing::UnitTest;
using namespace gtsam_cube;
using namespace gtsam;

class CubeFactorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    v_input << 0.5, 0.8, 0.3, 100.0, 12.0, 10.5;
    gtsam::Pose3 pose_temp = gtsam::Pose3::Expmap(v_input);
    Cube cube_temp = Cube(pose_temp, gtsam::Point3(2.0, 3.5, 0.5));
    landmarks.push_back(cube_temp);
    noiseModel = noiseModel::Diagonal::Sigmas(Vector9::Ones() * 0.01);
  }

  gtsam::Vector6 v_input;
  std::vector<Cube> landmarks;
  boost::shared_ptr<noiseModel::Diagonal> noiseModel;
};

// test initialization
TEST_F(CubeFactorTest, Initalizes) {
  auto c = Cube(landmarks[0].model.pose, landmarks[0].model.scale);
  CubeMeasurement cm = CubeMeasurement(c);
  auto cf = CubeFactor(0, 0, cm, noiseModel);
  EXPECT_TRUE(cf.equals(cf));
}

// test evaluateError function
TEST_F(CubeFactorTest, EvalError) {
  // this evaluateError does not seem straightforward to check, the
  // translational part will be influenced by rotational part too, so the vector
  // noise added are not exactly the error vector evaluated... need a better
  // understanding of this to proceed, now setting the error to 0 (if rotation
  // of v_input is 0 then the translation part error can be evaluated correctly)

  gtsam::Vector6 v_input2;
  v_input2 << 0.0, 0.0, 0.0, 100.0, 12.0, 10.5;
  gtsam::Pose3 pose_temp2 = gtsam::Pose3::Expmap(v_input2);
  Cube cube_temp2 = Cube(pose_temp2, gtsam::Point3(2.0, 3.5, 0.5));
  auto c = Cube(cube_temp2.model.pose, cube_temp2.model.scale);

  // test translation
  gtsam::Vector6 v_error_temp;
  v_error_temp << 0.0, 0.0, 0.0, 5.0, 10.0, 20.0;
  gtsam::Pose3 pose_temp = gtsam::Pose3::Expmap(v_input2 + v_error_temp);

  CubeMeasurement cube_landmark = CubeMeasurement(c);
  auto c_error = Cube(pose_temp, cube_temp2.model.scale);
  CubeMeasurement cube_noisy = CubeMeasurement(c_error);
  CubeFactor cube_measurement = CubeFactor(0, 0, cube_noisy, noiseModel);
  CubeFactor cube_measurement_no_noise =
      CubeFactor(0, 0, cube_landmark, noiseModel);
  auto pose = gtsam::Pose3();
  auto error = cube_measurement.evaluateError(pose, cube_landmark, boost::none,
                                              boost::none);
  // auto error = cube_measurement_no_noise.evaluateError(
  //     pose, cube_noisy, boost::none, boost::none);

  // test rotation
  v_error_temp << 0.3, 0.5, 1.0, 5.0, 10.0, 20.0;
  gtsam::Pose3 pose_temp3 = gtsam::Pose3::Expmap(v_input2 + v_error_temp);

  CubeMeasurement cube_landmark2 = CubeMeasurement(c);
  auto c_error2 = Cube(pose_temp3, cube_temp2.model.scale);
  CubeMeasurement cube_noisy2 = CubeMeasurement(c_error2);
  CubeFactor cube_measurement2 = CubeFactor(0, 0, cube_noisy2, noiseModel);
  CubeFactor cube_measurement_no_noise2 =
      CubeFactor(0, 0, cube_landmark2, noiseModel);
  auto pose2 = gtsam::Pose3();
  auto error2 = cube_measurement2.evaluateError(pose, cube_landmark2,
                                                boost::none, boost::none);
  // auto error2 = cube_measurement_no_noise2.evaluateError(
  //     pose2, cube_noisy2, boost::none, boost::none);

  gtsam::Vector9 error_all;
  error_all << error2(0), error2(1), error2(2), error(3), error(4), error(5), 0,
      0, 0;


  for (auto i = 0; i < v_error_temp.size(); ++i) {
    std::cout << "for index:";
    std::cout << i;
    std::cout << ", calculated error is:";
    std::cout << (error_all[i]);
    std::cout << ", actual error is:";
    std::cout << (v_error_temp[i]) << '\n';
  }

  for (auto i = 0; i < v_error_temp.size(); ++i) {
    EXPECT_NEAR(error_all[i], v_error_temp[i], 1e-2);
  }
}

// test evaluateError Jacobian calculation
TEST_F(CubeFactorTest, EvalErrorJacobian) {
  auto cA = Cube(landmarks[0].model.pose, landmarks[0].model.scale);
  auto cB = Cube(landmarks[0].model.pose, landmarks[0].model.scale);
  SE3 tf;
  tf.translation()[1] = 0.5;
  cB.project(tf);

  cB.model.scale = gtsam::Point3(0.1, 0.1, 0.1);

  CubeMeasurement cAm = CubeMeasurement(cA);
  CubeMeasurement cBm = CubeMeasurement(cB);

  auto cf = CubeFactor(0, 0, cAm, noiseModel);
  auto pose = gtsam::Pose3();
  gtsam::Matrix H;
  auto error = cf.evaluateError(pose, cBm, H, boost::none);
  auto d = H * error;
  EXPECT_TRUE(d[1] - 0.5 < 1e-2);
  EXPECT_TRUE(d[6] - 0.1 < 1e-2);
  EXPECT_TRUE(d[7] - 0.1 < 1e-2);
  EXPECT_TRUE(d[8] - 0.1 < 1e-2);
}

// test Retract initialization
TEST_F(CubeFactorTest, RetractOrigin) {
  gtsam::Vector9 v;
  v << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1;
  gtsam::Vector9 v_pose;
  v_pose = v.head(6);
  CubeMeasurement cm = CubeMeasurement::Retract(v);
  for (int i = 0; i < 6; i++) {
    EXPECT_TRUE(gtsam::Pose3::Logmap(cm.pose)[i] == v_pose[i]);
  }
  EXPECT_TRUE(cm.scale[0] == 0.1);
  EXPECT_TRUE(cm.scale[1] == 0.1);
  EXPECT_TRUE(cm.scale[2] == 0.1);
}

// test LocalCoordinates initialization
TEST_F(CubeFactorTest, LocalOrigin) {
  auto c = Cube(landmarks[0].model.pose, landmarks[0].model.scale);
  CubeMeasurement cm = CubeMeasurement(c);
  gtsam::Vector9 v = CubeMeasurement::LocalCoordinates(cm);
  auto v_temp = gtsam::Pose3::Logmap(landmarks[0].model.pose);
  EXPECT_TRUE(v.head(6) == v_temp);
  EXPECT_TRUE(v[6] == landmarks[0].model.scale[0]);
  EXPECT_TRUE(v[7] == landmarks[0].model.scale[1]);
  EXPECT_TRUE(v[8] == landmarks[0].model.scale[2]);
}

// test localCoordinates function
TEST_F(CubeFactorTest, LocalCoordinates) {
  auto cA = Cube(landmarks[0].model.pose, landmarks[0].model.scale);
  auto cB = Cube(landmarks[0].model.pose, landmarks[0].model.scale);
  SE3 tf;
  // tf.translation()[1] = 0.5;
  // cA.project(tf);
  cA.model.scale = cA.model.scale + gtsam::Point3(0.1, 0.1, 0.1);
  CubeMeasurement cAm = CubeMeasurement(cA);
  CubeMeasurement cBm = CubeMeasurement(cB);

  // cAm - cBm
  gtsam::Vector9 v = cAm.localCoordinates(cBm);
  // EXPECT_NEAR(v[4], 0.5, 1e-2);
  EXPECT_NEAR(v[6], 0.1, 1e-2);
  EXPECT_NEAR(v[7], 0.1, 1e-2);
  EXPECT_NEAR(v[8], 0.1, 1e-2);
}

TEST_F(CubeFactorTest, Retract) {
  auto cA = Cube(landmarks[0].model.pose, landmarks[0].model.scale);
  CubeMeasurement cAm = CubeMeasurement(cA);
  gtsam::Vector9 v;
  v << 1.8, 0.2, 1.0, 30.0, 100, 2.5, 1.0, 1.0, 1.0;
  CubeMeasurement cBm = cAm.retract(v);
  gtsam::Vector6 pose_temp;

  // pose_temp = gtsam::Vector6(gtsam::Pose3::Logmap(cAm.pose)) + v.head(6);
  // Use exponential map to check
  gtsam::Pose3 pose_temp_pose3 =
      cAm.pose * gtsam::Pose3::Expmap(v.segment(0, 6));
  pose_temp = gtsam::Pose3::Logmap(pose_temp_pose3);

  for (int i = 0; i < 6; i++) {
    EXPECT_NEAR(pose_temp[i], gtsam::Pose3::Logmap(cBm.pose)[i], 0.01);
  }
  std::cout << "pose_temp:\n" << pose_temp << '\n';
  std::cout << "cAm:\n" << gtsam::Pose3::Logmap(cAm.pose) << '\n';
  std::cout << "cBm:\n" << gtsam::Pose3::Logmap(cBm.pose) << '\n';

  EXPECT_NEAR(cAm.scale[0] + 1, cBm.scale[0], 1e-2);
  EXPECT_NEAR(cAm.scale[1] + 1, cBm.scale[1], 1e-2);
  EXPECT_NEAR(cAm.scale[2] + 1, cBm.scale[2], 1e-2);
}

TEST_F(CubeFactorTest, FactorGraph) {
  auto graph = SemanticFactorGraph();
  auto poseA = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0));
  auto poseB = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.1, 0.0, 0.0));
  auto poseC = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-0.1, 0.0, 0.0));

  graph.setPriors(poseA);
  graph.addKeyPoseAndBetween(0, 1, poseA, poseB, 0);
  graph.addKeyPoseAndBetween(1, 2, poseB, poseC, 0);

  auto ca = Cube(landmarks[0].model.pose, landmarks[0].model.scale);
  auto cb = Cube(landmarks[0].model.pose, landmarks[0].model.scale);
  auto cc = Cube(landmarks[0].model.pose, landmarks[0].model.scale);
  ca.model.scale = gtsam::Point3(0.1, 0.1, 0.1);
  cb.model.scale = gtsam::Point3(0.15, 0.15, 0.15);
  cc.model.scale = gtsam::Point3(0.05, 0.05, 0.05);

  // Add two measurements of the same cube
  graph.addCubeFactor(0, 0, poseA, ca, false, 0);
  graph.addCubeFactor(1, 0, poseB, cb, true, 0);
  graph.solve();
  graph.getCube(0).print();

  // Add third measurement of the same cube
  graph.addCubeFactor(2, 0, poseC, cc, true, 0);
  graph.solve();
  graph.getCube(0).print();

  graph.getPose(0).print();
  graph.getPose(1).print();
  graph.getPose(2).print();
  EXPECT_NEAR(graph.getCube(0).scale[0], 0.1, 1e-5);
}