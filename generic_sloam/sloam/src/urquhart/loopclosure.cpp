#include <pcl/io/pcd_io.h>

#include <loopclosure.hpp>

namespace loop {

bool UrquhartLoopCloser::estimateTransformation(const CloudT::Ptr pA,
                                                const CloudT::Ptr pB, SE3 &tf) {
  pcl::registration::TransformationEstimationLM<PointT, PointT>::Ptr te(
      new pcl::registration::TransformationEstimationLM<PointT, PointT>);
  Eigen::Matrix4f tf_matrix;
  te->estimateRigidTransformation(*pA, *pB, tf_matrix);

  if (isOrthogonal(tf_matrix)) {
    Sophus::SE3f ttff = Sophus::SE3f(tf_matrix);
    tf = Sophus::SE3d(ttff.cast<double>());
    return true;
  }

  return false;
}

double UrquhartLoopCloser::computeResidual(const CloudT::Ptr src_pc,
                                           const CloudT::Ptr tgt_pc,
                                           const SE3 &tf) {
  CloudT::Ptr aux_pc = CloudT::Ptr(new CloudT);
  pcl::transformPointCloud(*src_pc, *aux_pc, tf.matrix());

  double r = 0;
  for (auto i = 0; i < tgt_pc->points.size(); ++i) {
    double step = euclideanDistance(tgt_pc->points[i], aux_pc->points[i]);
    r += step;
  }

  return r / tgt_pc->points.size();
}

bool UrquhartLoopCloser::ICP(const CloudT::Ptr &src_pc,
                             const CloudT::Ptr &tgt_pc, SE3 &tf) {
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(200);
  icp.setMaxCorrespondenceDistance(1.5);
  icp.setTransformationEpsilon(1e-9);
  icp.setRANSACOutlierRejectionThreshold(0.1);
  // icp.setEuclideanFitnessEpsilon(0.001);
  icp.setRANSACIterations(100);

  // Align clouds
  icp.setInputSource(src_pc);
  icp.setInputTarget(tgt_pc);
  CloudT::Ptr unused_result(new CloudT());
  icp.align(*unused_result);

  if (icp.hasConverged() == false) {
    ROS_INFO("%f:", icp.getFitnessScore());
    return false;
  }

  auto affine = icp.getFinalTransformation();
  ROS_DEBUG_STREAM("ICP: \n" << affine);

  if (isOrthogonal(affine)) {
    Sophus::SE3f ttff = Sophus::SE3f(affine);
    tf = Sophus::SE3d(ttff.cast<double>());
    return true;
  }
  return false;
}

bool UrquhartLoopCloser::poseOptimization(const std::vector<Cylinder> &source,
                                          const std::vector<Cylinder> &target,
                                          const std::vector<idxPair> &matches,
                                          SE3 &tf) {
  SE3 ransac_tf = SE3();
  double ransac_residual = max_r_;
  bool ransac_res = RANSAC(source, target, matches, ransac_residual, ransac_tf);

  if (ransac_res) {
    CloudT::Ptr src_pc = CloudT::Ptr(new CloudT);
    CloudT::Ptr tgt_pc = CloudT::Ptr(new CloudT);
    ROS_DEBUG("MAKING PCS");
    makeCylinderPointClouds(matches, source, target, ransac_tf,
                            ransac_residual * 2, src_pc, tgt_pc);

    // CHECK IF PCS are EMPTY
    if (src_pc->points.size() > 0) {
      ROS_DEBUG("ICP");
      // perform ICP
      SE3 icp_tf = SE3();
      bool converged = ICP(src_pc, tgt_pc, icp_tf);

      if (converged) {
        tf = icp_tf * ransac_tf;
        ROS_DEBUG_STREAM("FINAL: \n" << tf.matrix());
        return true;
      }
    }
  }

  return false;
}

bool UrquhartLoopCloser::RANSAC(const std::vector<Cylinder> &source,
                                const std::vector<Cylinder> &target,
                                const std::vector<idxPair> &matches,
                                double &residual, SE3 &tf) {
  int ransac_iters_ = 100;
  int ransac_samples_ = 8;
  double best_residual = 100000;
  for (auto i = 0; i < ransac_iters_; ++i) {
    // sample random associations
    std::vector<idxPair> sampledMatches;
    std::sample(matches.begin(), matches.end(),
                std::back_inserter(sampledMatches), ransac_samples_,
                std::mt19937{std::random_device{}()});

    // create point cloud from associated cylinders
    CloudT::Ptr src_pc = CloudT::Ptr(new CloudT);
    CloudT::Ptr tgt_pc = CloudT::Ptr(new CloudT);
    makePointClouds(sampledMatches, source, target, src_pc, tgt_pc);
    // Estimate tf using cylidner points
    SE3 temp_tf;
    estimateTransformation(src_pc, tgt_pc, temp_tf);
    double r = computeResidual(src_pc, tgt_pc, temp_tf);
    // ROS_DEBUG_STREAM("Iter Residual " << r);
    if (r < best_residual) {
      best_residual = r;
      tf = temp_tf;
    }
  }

  ROS_DEBUG_STREAM("Best Residual " << best_residual);
  ROS_DEBUG_STREAM("RANSAC: \n" << tf.matrix());
  residual = best_residual;
  if (best_residual > max_r_) return false;
  return true;
}

bool UrquhartLoopCloser::tryLoopClosure(const std::vector<Cylinder> &source,
                                        const std::vector<Cylinder> &target,
                                        SE3 &tf) {
  if (source.size() < 4 || target.size() < 4) return false;

  auto pA = filterPoints(makePointVector(source));
  auto pB = filterPoints(makePointVector(target));

  urquhart::Observation obsA(pA);
  urquhart::Observation obsB(pB);
  std::vector<idxPair> idxMatches;
  matching::hierarchyMatching(obsA, obsB, 5, boost::none, idxMatches);
  if (idxMatches.size() < min_matches_) return false;

  if (poseOptimization(source, target, idxMatches, tf)) return true;

  return false;
}

}  // namespace loop