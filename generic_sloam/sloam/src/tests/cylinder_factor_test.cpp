#include <cylinder.h>
#include <cylinderFactor.h>
#include <definitions.h>
#include <graph.h>
#include <plane.h>
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
using namespace gtsam_cylinder;
using namespace gtsam;

class CylinderFactorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    setupParams();
    PointT pta, ptb, ptc, ptd;
    pta.x = 0;
    pta.y = 0;
    pta.z = 0;
    ptb.x = 0;
    ptb.y = 1;
    ptb.z = 0;
    ptc.x = 1;
    ptc.y = 0;
    ptc.z = 0;
    ptd.x = 0.5;
    ptd.y = 0;
    ptd.z = 0;
    VectorType gfeatures{pta, ptb, ptc, ptd};
    plane = boost::make_shared<Plane>(gfeatures, params);

    std::ifstream ifs(
        "/opt/generic-sloam_ws/src/sloam/src/tests/aux/still_landmarks_t0");
    boost::archive::text_iarchive ia(ifs);
    // read class state from archive
    ia >> landmarks;
    noiseModel = noiseModel::Diagonal::Sigmas(Vector7::Ones() * 0.01);
  }

  void setupParams() {
    params.scansPerSweep = 1;
    params.maxLidarDist = 30.0;
    params.maxGroundLidarDist = 30.0;
    params.minGroundLidarDist = 0.0;

    params.groundRadiiBins = 1;
    params.groundThetaBins = 1;
    params.groundMatchThresh = 2.0;
    params.groundRetainThresh = 0.1;

    params.maxTreeRadius = 0.3;
    params.maxAxisTheta = 10;
    params.maxFocusOutlierDistance = 0.5;
    params.roughTreeMatchThresh = 3.0;
    params.treeMatchThresh = 1.0;

    params.AddNewTreeThreshDist = 2.0;

    params.featuresPerTree = 2;
    params.numGroundFeatures = 3;
    params.defaultTreeRadius = 0.1;
  }

  std::vector<std::vector<TreeVertex>> landmarks;
  FeatureModelParams params;
  boost::shared_ptr<Plane> plane;
  boost::shared_ptr<noiseModel::Diagonal> noiseModel;
};

TEST_F(CylinderFactorTest, Initalizes) {
  auto c = Cylinder(landmarks[0], *plane, params);
  CylinderMeasurement cm = CylinderMeasurement(c);
  auto cf = CylinderFactor(0, 0, cm, noiseModel);
  EXPECT_TRUE(cf.equals(cf));
}

TEST_F(CylinderFactorTest, EvalError) {
  auto c = Cylinder(landmarks[0], *plane, params);
  CylinderMeasurement cm = CylinderMeasurement(c);
  auto cf = CylinderFactor(0, 0, cm, noiseModel);
  auto pose = gtsam::Pose3();
  auto error = cf.evaluateError(pose, cm, boost::none, boost::none);
  for (auto i = 0; i < error.size(); ++i) EXPECT_TRUE(error[i] < 1e-7);
}

TEST_F(CylinderFactorTest, EvalErrorJacobian) {
  auto cA = Cylinder(landmarks[0], *plane, params);
  auto cB = Cylinder(landmarks[0], *plane, params);
  SE3 tf;
  tf.translation()[1] = 0.5;
  cB.project(tf);
  cB.model.radius = 0.2;

  CylinderMeasurement cAm = CylinderMeasurement(cA);
  CylinderMeasurement cBm = CylinderMeasurement(cB);

  auto cf = CylinderFactor(0, 0, cAm, noiseModel);
  auto pose = gtsam::Pose3();
  gtsam::Matrix H;
  auto error = cf.evaluateError(pose, cBm, H, boost::none);
  auto d = H * error;
  EXPECT_TRUE(d[1] - 0.5 < 1e-2);
  EXPECT_TRUE(d[6] - 0.1 < 1e-2);
}

TEST_F(CylinderFactorTest, RetractOrigin) {
  gtsam::Vector7 v;
  v << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1;
  CylinderMeasurement cm = CylinderMeasurement::Retract(v);
  EXPECT_TRUE(cm.root[0] == 0);
  EXPECT_TRUE(cm.root[1] == 0);
  EXPECT_TRUE(cm.root[2] == 0);
}

TEST_F(CylinderFactorTest, LocalOrigin) {
  auto c = Cylinder(landmarks[0], *plane, params);
  CylinderMeasurement cm = CylinderMeasurement(c);
  gtsam::Vector7 v = CylinderMeasurement::LocalCoordinates(cm);
  EXPECT_TRUE(v[4] > 10.0);
  EXPECT_TRUE(v[6] == 0.1);
}

TEST_F(CylinderFactorTest, LocalCoordinates) {
  auto cA = Cylinder(landmarks[0], *plane, params);
  auto cB = Cylinder(landmarks[0], *plane, params);
  SE3 tf;
  tf.translation()[1] = 0.5;
  cB.project(tf);
  cB.model.radius = 0.2;

  CylinderMeasurement cAm = CylinderMeasurement(cA);
  CylinderMeasurement cBm = CylinderMeasurement(cB);

  gtsam::Vector7 v = cAm.localCoordinates(cBm);
  EXPECT_TRUE(v[4] - 0.5 < 1e-2);
  EXPECT_TRUE(v[6] - 0.1 < 1e-2);
}

TEST_F(CylinderFactorTest, Retract) {
  auto cA = Cylinder(landmarks[0], *plane, params);
  CylinderMeasurement cAm = CylinderMeasurement(cA);
  gtsam::Vector7 v;
  v << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1;
  CylinderMeasurement cBm = cAm.retract(v);
  EXPECT_NEAR(cAm.root[0] + 1, cBm.root[0], 1e-2);
}

TEST_F(CylinderFactorTest, FactorGraph) {
  auto graph = SemanticFactorGraph();
  auto poseA = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0));
  auto poseB = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.1, 0.0, 0.0));
  auto poseC = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-0.1, 0.0, 0.0));

  graph.setPriors(poseA);
  graph.addKeyPoseAndBetween(0, 1, poseA, poseB, 0);
  graph.addKeyPoseAndBetween(1, 2, poseB, poseC, 0);

  auto ca = Cylinder(landmarks[0], *plane, params);
  auto cb = Cylinder(landmarks[0], *plane, params);
  auto cc = Cylinder(landmarks[0], *plane, params);
  ca.model.radius = 0.1;
  cb.model.radius = 0.15;
  cc.model.radius = 0.05;

  // Add two measurements of the same cylinder
  graph.addCylinderFactor(0, 0, poseA, ca, false);
  graph.addCylinderFactor(1, 0, poseB, cb, true);
  graph.solve();
  graph.getCylinder(0).print();

  // Add third measurement of the same cylinder
  graph.addCylinderFactor(2, 0, poseC, cc, true);
  graph.solve();
  graph.getCylinder(0).print();

  graph.getPose(0).print();
  graph.getPose(1).print();
  graph.getPose(2).print();
  EXPECT_NEAR(graph.getCylinder(0).radius, 0.1, 1e-5);
}