#include <cylinder.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_lm.h>

#include <cloud_utils.hpp>
#include <distance.hpp>
#include <matching.hpp>
#include <observation.hpp>
#include <sophus/geometry.hpp>
#include <sophus/se3.hpp>

namespace loop {

struct LoopFactorData {
  LoopFactorData(const SE3 &p, const size_t idxF, const size_t idxT)
      : betweenPose(p), idxFrom(idxF), idxTo(idxT){};
  SE3 betweenPose;
  size_t idxFrom;
  size_t idxTo;
};

class UrquhartLoopCloser {
 public:
  explicit UrquhartLoopCloser(int min_matches, double total_residual)
      : min_matches_(min_matches), max_r_(total_residual){};
  bool tryLoopClosure(const std::vector<Cylinder> &source,
                      const std::vector<Cylinder> &target, SE3 &tf);

 private:
  bool poseOptimization(const std::vector<Cylinder> &source,
                        const std::vector<Cylinder> &target,
                        const std::vector<idxPair> &matches, SE3 &tf);
  bool ICP(const CloudT::Ptr &src_pc, const CloudT::Ptr &tgt_pc, SE3 &tf);
  bool RANSAC(const std::vector<Cylinder> &source,
              const std::vector<Cylinder> &target,
              const std::vector<idxPair> &matches, double &residual, SE3 &tf);

  bool estimateTransformation(const CloudT::Ptr pA, const CloudT::Ptr pB,
                              SE3 &tf);
  double computeResidual(const CloudT::Ptr src_pc, const CloudT::Ptr tgt_pc,
                         const SE3 &tf);

  double max_r_;
  int min_matches_;
  int count = 0;
};

}  // namespace loop