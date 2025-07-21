#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <string>
#include <list>
#include <utility>
#include <unordered_map>
#include <angles/angles.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "active_perception/cop_utils.h"

#include "cop_lib/cop.h"
#include "cop_lib/typedefs.h"
#include "cop_lib/graph.h"
#include "cop_lib/apsp_floyd_warshall.h"

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
using std::list;
using std::pair;

class RayCaster;

namespace ms_planner {
class EDTEnvironment;
class PerceptionUtils;
#define COST_INF 100000;


// Viewpoint to cover a frontier cluster
struct Viewpoint {
  Vector3d pos_;
  double yaw_;
  int visib_num_;
  int info_gain_;
};

// A frontier cluster, the viewpoints to cover it
struct Frontier {
  // Complete voxels belonging to the cluster
  vector<Vector3d> cells_;
  // down-sampled voxels filtered by voxel grid filter
  vector<Vector3d> filtered_cells_;
  // Average position of all voxels
  Vector3d average_;
  // Idx of cluster
  int id_;
  // Viewpoints that can cover the cluster
  vector<Viewpoint> viewpoints_;
  // Bounding box of cluster, center & 1/2 side length
  Vector3d box_min_, box_max_;
  list<vector<Vector3d>> paths_;
  list<double> costs_;
};


class FrontierFinder {
public:
  FrontierFinder(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh);
  ~FrontierFinder();
  double evaFrontierChanged();

  // Update Frontiers: remove updated/old frontiers
  void updateFrontiers();

  // Remove old frontiers and search for new ones
  void searchFrontiers();
  void computeFrontiersToVisitIGNoPred();

  //get frontier
  void getFrontierObjs(list<Frontier> &cluster_objs);
  void getFrontiers(vector<vector<Vector3d>>& clusters);
  void getDormantFrontiers(vector<vector<Vector3d>>& clusters);
  void getFrontierBoxes(vector<pair<Vector3d, Vector3d>>& boxes);
  // Get viewpoint with highest coverage for each frontier
  void getTopViewpointsIGNoPred(const Vector3d& cur_pos, vector<Eigen::Vector3d>& points, vector<double>& yaws, 
                            vector<Eigen::Vector3d>& averages, vector<int>& info_gains);
  void updateFrontierCostMatrix();
  void getFullCostMatrix(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
                         Eigen::MatrixXd& mat);
  void getPathForTour(const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path);
  void getPathForPlannedTour(const Vector3d& pos, const vector<int>& planned_frontier_ids, vector<Vector3d>& path);
  void getPathForRefinedTour(const Vector3d& pos, const vector<std::string>& refined_sequence, 
                            const std::unordered_map<int, Eigen::Vector3d>& key_pose_pos, vector<Vector3d>& path);
  void setNextFrontier(const int& id);
  bool isFrontierCovered();
  void wrapYaw(double& yaw);
  

  int computeYawInfoGain(const Eigen::Vector3d& pos, 
                         const double& yaw);

  bool room_visited_;
  
  shared_ptr<PerceptionUtils> percep_utils_;


  int occSearch(const Vector3d& pos);

  void getCorrelationMatrix(const Vector3d& pos, const double& yaw, vector<vector<double>>& correlations);
  double getNodeCorrelation(const Viewpoint& vp1, const Viewpoint& vp2);

  void copyFrontierToPlanning();
  void copyFrontierToPlanned();
  bool global_tour_planned;

  void checkParam();

private:
  void splitLargeFrontiers(list<Frontier>& frontiers);
  bool splitHorizontally(const Frontier& frontier, list<Frontier>& splits);
  void mergeFrontiers(Frontier& ftr1, const Frontier& ftr2);
  bool isFrontierChanged(const Frontier& ft);
  bool haveOverlap(const Vector3d& min1, const Vector3d& max1, const Vector3d& min2,
                   const Vector3d& max2);
  void computeFrontierInfo(Frontier& frontier);
  void downsample(const vector<Vector3d>& cluster_in, vector<Vector3d>& cluster_out);
  void sampleViewpoints(Frontier& frontier);
  void sampleViewpointsIG(Frontier& frontier);
  void sampleViewpointsIGNoPred(Frontier& frontier);
  void sampleViewpointsClassic(Frontier& frontier);

  void computeInfoGainNoPred(const Eigen::Vector3d& pos, const double& yaw, vector<Viewpoint>& vps);
  bool isNearUnknown(const Vector3d& pos);
  bool isNearObstacle(const Vector3d& pos);
  vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel);
  bool isNeighborUnknown(const Eigen::Vector3i& voxel);
  void expandFrontier(const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */);

  // Wrapper of sdf map
  int toadr(const Eigen::Vector3i& idx);
  bool knownfree(const Eigen::Vector3i& idx);
  bool inmap(const Eigen::Vector3i& idx);

  // Deprecated
  Eigen::Vector3i searchClearVoxel(const Eigen::Vector3i& pt);
  bool isInBoxes(const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx);
  bool canBeMerged(const Frontier& ftr1, const Frontier& ftr2);
  void findViewpoints(const Vector3d& sample, const Vector3d& ftr_avg, vector<Viewpoint>& vps);

  // Data
  vector<char> frontier_flag_;
  list<Frontier> frontiers_, dormant_frontiers_, tmp_frontiers_, planning_frontiers_, planned_frontiers_;
  vector<int> removed_ids_;
  list<Frontier>::iterator first_new_ftr_;
  Frontier next_frontier_;

  // Params
  int cluster_min_;
  double cluster_size_xy_, cluster_size_z_;
  double candidate_rmax_, candidate_rmin_, candidate_dphi_, min_candidate_dist_, room_min_candidate_dist_,
      min_candidate_clearance_, min_candidate_obs_clearance_;
  int down_sample_;
  double min_view_finish_fraction_, resolution_;
  int min_visib_num_, candidate_rnum_;
  int min_info_gain_;
  double v_fov_, h_fov_, min_ray_length_, max_ray_length_;


  Vector3d cur_pos_;
  double cur_yaw_;

  // Utils
  shared_ptr<EDTEnvironment> edt_env_;
  unique_ptr<RayCaster> raycaster_;

  // Debug param
  bool debug_;

};

}  // namespace ms_planner
#endif