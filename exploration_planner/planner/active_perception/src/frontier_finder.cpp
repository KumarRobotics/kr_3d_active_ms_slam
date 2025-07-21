#include <active_perception/frontier_finder.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_env/edt_environment.h>
#include <active_perception/perception_utils.h>
#include <active_perception/graph_node.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Eigenvalues>

namespace ms_planner {
FrontierFinder::FrontierFinder(const EDTEnvironment::Ptr& edt, ros::NodeHandle& nh) {
  this->edt_env_ = edt;
  int voxel_num = edt->occ_map_->getVoxelNum();
  frontier_flag_ = vector<char>(voxel_num, 0);
  fill(frontier_flag_.begin(), frontier_flag_.end(), 0);

  nh.param("frontier/cluster_min", cluster_min_, -1);
  nh.param("frontier/cluster_size_xy", cluster_size_xy_, -1.0);
  nh.param("frontier/cluster_size_z", cluster_size_z_, -1.0);
  nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
  nh.param("frontier/room_min_candidate_dist", room_min_candidate_dist_, -1.0);
  nh.param("frontier/min_candidate_clearance", min_candidate_clearance_, -1.0);
  nh.param("frontier/min_candidate_obs_clearance", min_candidate_obs_clearance_, -1.0);
  nh.param("frontier/candidate_dphi", candidate_dphi_, -1.0);
  nh.param("frontier/candidate_rmax", candidate_rmax_, -1.0);
  nh.param("frontier/candidate_rmin", candidate_rmin_, -1.0);
  nh.param("frontier/candidate_rnum", candidate_rnum_, -1);
  nh.param("frontier/down_sample", down_sample_, -1);
  nh.param("frontier/min_visib_num", min_visib_num_, -1);
  nh.param("frontier/min_info_gain", min_info_gain_, -1);
  nh.param("frontier/min_view_finish_fraction", min_view_finish_fraction_, -1.0);
  nh.param("frontier/h_fov", h_fov_, -1.0);
  nh.param("frontier/v_fov", v_fov_, -1.0);
  nh.param("frontier/min_ray_length", min_ray_length_, -1.0);
  nh.param("frontier/max_ray_length", max_ray_length_, -1.0);

  nh.param("debug_print", debug_, false);

  raycaster_.reset(new RayCaster);
  resolution_ = edt_env_->occ_map_->getResolution();
  Eigen::Vector3d origin, size;
  edt_env_->occ_map_->getRegion(origin, size);
  raycaster_->setParams(resolution_, origin);

  percep_utils_.reset(new PerceptionUtils(nh));

  removed_ids_.clear();
  global_tour_planned = false;
}

FrontierFinder::~FrontierFinder() {
}

void FrontierFinder::checkParam() {
  // Check
  ViewNode::caster_->checkParams();
}

void FrontierFinder::updateFrontiers()
{
  // Bounding box of updated region
  Vector3d update_min, update_max;
  edt_env_->occ_map_->getUpdatedBox(update_min, update_max, true);
  // Removed changed frontiers in updated map 
  // Remove changed frontiers' buffers as well
  // Changed frontier means visited frontier, make it a topo node in topo map
  auto resetFlag = [&](list<Frontier>::iterator& iter, list<Frontier>& frontiers) {
    Eigen::Vector3i idx;
    for (auto cell : iter->cells_) {
      edt_env_->occ_map_->posToIndex(cell, idx);
      frontier_flag_[toadr(idx)] = 0;
    }
    iter = frontiers.erase(iter);
  };

  std::cout << "[FrontierFinder-searchFrontiers] Before remove: " << frontiers_.size() << std::endl;
  //removed_ids_.clear();
  int rmv_idx = 0;
  for (auto iter = frontiers_.begin(); iter != frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isFrontierChanged(*iter)) {
      resetFlag(iter, frontiers_);
      removed_ids_.push_back(rmv_idx);
    } else {
      ++rmv_idx;
      ++iter;
    }
  }
 
  for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isFrontierChanged(*iter))
      resetFlag(iter, dormant_frontiers_);
    else
      ++iter;
  }
  std::cout << "[FrontierFinder-searchFrontiers] After remove: " << frontiers_.size() << std::endl;
  return;
}

void FrontierFinder::searchFrontiers() {
  ros::Time t1 = ros::Time::now();
  tmp_frontiers_.clear();

  // Bounding box of updated region
  Vector3d update_min, update_max;
  edt_env_->occ_map_->getUpdatedBox(update_min, update_max, true);
  updateFrontiers();

  // Note: here, the updated box will either be update_min, update_max if it is updated,
  // Otherwise, it will be camera_pos
 // Search new frontier within box slightly inflated from updated box
  Vector3d search_min = update_min - Vector3d(1, 1, 0.5);
  Vector3d search_max = update_max + Vector3d(1, 1, 0.5);
  Vector3d box_min, box_max;
  edt_env_->occ_map_->getBox(box_min, box_max);
  for (int k = 0; k < 3; ++k) {
    search_min[k] = max(search_min[k], box_min[k]);
    search_max[k] = min(search_max[k], box_max[k]);
  }
  Eigen::Vector3i min_id, max_id;
  edt_env_->occ_map_->posToIndex(search_min, min_id);
  edt_env_->occ_map_->posToIndex(search_max, max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        // Scanning the updated region to find seeds of frontiers
        Eigen::Vector3i cur(x, y, z);
        // If in no flight zone, skip
        if (edt_env_->occ_map_->mp_->set_no_flight_zone_) {
          if (edt_env_->occ_map_->isInNoFlightZone(cur))
            continue;
        }

        if (frontier_flag_[toadr(cur)] == 0 && knownfree(cur) && isNeighborUnknown(cur)) {
          // Expand from the seed cell to find a complete frontier cluster
          expandFrontier(cur);
        }
      }
  splitLargeFrontiers(tmp_frontiers_);

  ROS_WARN_THROTTLE(5.0, "Frontier t: %lf", (ros::Time::now() - t1).toSec());
}


void FrontierFinder::expandFrontier(
    const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */) {
  // std::cout << "depth: " << depth << std::endl;
  auto t1 = ros::Time::now();

  // Data for clustering
  queue<Eigen::Vector3i> cell_queue;
  vector<Eigen::Vector3d> expanded;
  Vector3d pos;

  edt_env_->occ_map_->indexToPos(first, pos);
  expanded.push_back(pos);
  cell_queue.push(first);
  frontier_flag_[toadr(first)] = 1;

  // Search frontier cluster based on region growing (distance clustering)
  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    auto nbrs = allNeighbors(cur);
    for (auto nbr : nbrs) {
      // Qualified cell should be inside bounding box and frontier cell not clustered
      int adr = toadr(nbr);
      if (frontier_flag_[adr] == 1 || !edt_env_->occ_map_->isInBox(nbr) ||
          !(knownfree(nbr) && isNeighborUnknown(nbr)))
        continue;

      edt_env_->occ_map_->indexToPos(nbr, pos);
      // Note: Frontiers won't be found from z < 0.4
      if (pos[2] < 0.4) continue;  // Remove noise close to ground
      expanded.push_back(pos);
      cell_queue.push(nbr);
      frontier_flag_[adr] = 1;
    }
  }
  if (expanded.size() > cluster_min_) {
    // Compute detailed info
    Frontier frontier;
    frontier.cells_ = expanded;
    computeFrontierInfo(frontier);
    tmp_frontiers_.push_back(frontier);
  }
}

void FrontierFinder::splitLargeFrontiers(list<Frontier>& frontiers) {
  list<Frontier> splits, tmps;
  for (auto it = frontiers.begin(); it != frontiers.end(); ++it) {
    // Check if each frontier needs to be split horizontally
    if (splitHorizontally(*it, splits)) {
      tmps.insert(tmps.end(), splits.begin(), splits.end());
      splits.clear();
    } else
      tmps.push_back(*it);
  }
  frontiers = tmps;
}

bool FrontierFinder::splitHorizontally(const Frontier& frontier, list<Frontier>& splits) {
  // Split a frontier into small piece if it is too large
  auto mean = frontier.average_.head<2>();
  bool need_split = false;

  // any cell's distance to center greater than cluster size
  for (auto cell : frontier.filtered_cells_) {
    if ((cell.head<2>() - mean).norm() > cluster_size_xy_) {
      need_split = true;
      break;
    }
  }
  if (!need_split) return false;

  // Compute principal component
  // Covariance matrix of cells
  Eigen::Matrix2d cov;
  cov.setZero();
  for (auto cell : frontier.filtered_cells_) {
    Eigen::Vector2d diff = cell.head<2>() - mean;
    cov += diff * diff.transpose();
  }
  cov /= double(frontier.filtered_cells_.size());

  // Find eigenvector corresponds to maximal eigenvector
  Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int max_idx;
  double max_eigenvalue = -1000000;
  for (int i = 0; i < values.rows(); ++i) {
    if (values[i] > max_eigenvalue) {
      max_idx = i;
      max_eigenvalue = values[i];
    }
  }
  Eigen::Vector2d first_pc = vectors.col(max_idx);
  // std::cout << "max idx: " << max_idx << std::endl;
  // std::cout << "mean: " << mean.transpose() << ", first pc: " << first_pc.transpose() << std::endl;

  // Split the frontier into two groups along the first PC
  Frontier ftr1, ftr2;
  for (auto cell : frontier.cells_) {
    if ((cell.head<2>() - mean).dot(first_pc) >= 0)
      ftr1.cells_.push_back(cell);
    else
      ftr2.cells_.push_back(cell);
  }
  computeFrontierInfo(ftr1);
  computeFrontierInfo(ftr2);

  // Recursive call to split frontier that is still too large
  list<Frontier> splits2;
  if (splitHorizontally(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  } else
    splits.push_back(ftr1);

  if (splitHorizontally(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

bool FrontierFinder::isInBoxes(
    const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx) {
  Vector3d pt;
  edt_env_->occ_map_->indexToPos(idx, pt);
  for (auto box : boxes) {
    // Check if contained by a box
    bool inbox = true;
    for (int i = 0; i < 3; ++i) {
      inbox = inbox && pt[i] > box.first[i] && pt[i] < box.second[i];
      if (!inbox) break;
    }
    if (inbox) return true;
  }
  return false;
}

void FrontierFinder::updateFrontierCostMatrix() {
  std::cout << "cost mat size before remove: " << std::endl;
  for (auto ftr : frontiers_)
    std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
  std::cout << "" << std::endl;

  if (!removed_ids_.empty())
  {
    for (int i = 0; i < removed_ids_.size(); ++i)
    {std::cout << "(" << removed_ids_[i] << "), ";}
  }else
  {
    std::cout << "removed_ids_ is empty " << std::endl;
  }


  // std::cout << "cost mat size remove: " << std::endl;
  if (!removed_ids_.empty()) {
    // Delete path and cost for removed clusters
    for (auto it = frontiers_.begin(); it != first_new_ftr_; ++it) {
      auto cost_iter = it->costs_.begin();
      auto path_iter = it->paths_.begin();
      int iter_idx = 0;
      for (int i = 0; i < removed_ids_.size(); ++i) {
        // Step iterator to the item to be removed
        while (iter_idx < removed_ids_[i]) {
          ++cost_iter;
          ++path_iter;
          ++iter_idx;
        }
        cost_iter = it->costs_.erase(cost_iter);
        path_iter = it->paths_.erase(path_iter);
      }
    }
    removed_ids_.clear();
  }

  auto updateCost = [](const list<Frontier>::iterator& it1, const list<Frontier>::iterator& it2) {
    ROS_WARN_THROTTLE(1, "Enter update cost function");
    // Search path from old cluster's top viewpoint to new cluster'
    Viewpoint& vui = it1->viewpoints_.front();
    Viewpoint& vuj = it2->viewpoints_.front();
    vector<Vector3d> path_ij;
    double cost_ij = ViewNode::computeCost(
        vui.pos_, vuj.pos_, vui.yaw_, vuj.yaw_, Vector3d(0, 0, 0), 0, path_ij);
    // Insert item for both old and new clusters
    it1->costs_.push_back(cost_ij);
    it1->paths_.push_back(path_ij);
    reverse(path_ij.begin(), path_ij.end());
    it2->costs_.push_back(cost_ij);
    it2->paths_.push_back(path_ij);
  };

  // Compute path and cost between old and new clusters
  for (auto it1 = frontiers_.begin(); it1 != first_new_ftr_; ++it1) {
    for (auto it2 = first_new_ftr_; it2 != frontiers_.end(); ++it2) {
      updateCost(it1, it2);
    }
  }

  // Compute path and cost between new clusters
  for (auto it1 = first_new_ftr_; it1 != frontiers_.end(); ++it1) {
    for (auto it2 = it1; it2 != frontiers_.end(); ++it2) {
      if (it1 == it2) {
        std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
        it1->costs_.push_back(0);
        it1->paths_.push_back({});
      } else
        updateCost(it1, it2);
    }
  }

  // std::cout << "" << std::endl;
  // std::cout << "cost mat size final: " << std::endl;
  // for (auto ftr : frontiers_)
  //   std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
  // std::cout << "" << std::endl;
}

void FrontierFinder::mergeFrontiers(Frontier& ftr1, const Frontier& ftr2) {
  // Merge ftr2 into ftr1
  ftr1.average_ =
      (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) /
      (double(ftr1.cells_.size() + ftr2.cells_.size()));
  ftr1.cells_.insert(ftr1.cells_.end(), ftr2.cells_.begin(), ftr2.cells_.end());
  computeFrontierInfo(ftr1);
}

bool FrontierFinder::canBeMerged(const Frontier& ftr1, const Frontier& ftr2) {
  Vector3d merged_avg =
      (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) /
      (double(ftr1.cells_.size() + ftr2.cells_.size()));
  // Check if it can merge two frontier without exceeding size limit
  for (auto c1 : ftr1.cells_) {
    auto diff = c1 - merged_avg;
    if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_) return false;
  }
  for (auto c2 : ftr2.cells_) {
    auto diff = c2 - merged_avg;
    if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_) return false;
  }
  return true;
}

bool FrontierFinder::haveOverlap(
    const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2) {
  // Check if two box have overlap part
  Vector3d bmin, bmax;
  for (int i = 0; i < 3; ++i) {
    bmin[i] = max(min1[i], min2[i]);
    bmax[i] = min(max1[i], max2[i]);
    if (bmin[i] > bmax[i] + 1e-3) return false;
  }
  return true;
}

bool FrontierFinder::isFrontierChanged(const Frontier& ft) {
  for (auto cell : ft.cells_) {
    Eigen::Vector3i idx;
    edt_env_->occ_map_->posToIndex(cell, idx);
    if (!(knownfree(idx) && isNeighborUnknown(idx))) return true; // 
  }
  return false;
}



double FrontierFinder::evaFrontierChanged() {
  
  double num1 = frontiers_.size(), num2 = 0.0;
  
  if(num1 == 0) return 0;
  for (auto iter = frontiers_.begin(); iter != frontiers_.end();) {
    if (isFrontierChanged(*iter)) {
      ++num2;
     
    }
    ++iter;
  }
  std::cout << "frontier changed : " << num2 / num1 << std::endl;
  return num2 / num1;
}


void FrontierFinder::computeFrontierInfo(Frontier& ftr) {
  // Compute average position and bounding box of cluster
  ftr.average_.setZero();
  ftr.box_max_ = ftr.cells_.front();
  ftr.box_min_ = ftr.cells_.front();
  for (auto cell : ftr.cells_) {
    ftr.average_ += cell;
    for (int i = 0; i < 3; ++i) {
      ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
      ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
    }
  }
  ftr.average_ /= double(ftr.cells_.size());
  // Compute downsampled cluster
  // make it sparse, for vis?, for less computation?
  downsample(ftr.cells_, ftr.filtered_cells_);
}



void FrontierFinder::computeFrontiersToVisitIGNoPred() {
  first_new_ftr_ = frontiers_.end();
  int new_num = 0;
  int new_dormant_num = 0;
  // sample viewpoint from tmp_frontiers, insert to frontiers_
  // Try find viewpoints for each cluster and categorize them according to viewpoint number
  for (auto& tmp_ftr : tmp_frontiers_) {
    // Search viewpoints around frontier
    sampleViewpointsIGNoPred(tmp_ftr);
    if (!tmp_ftr.viewpoints_.empty()) {
      ++new_num;
      // iterator points to the first of the newly inserted element
      list<Frontier>::iterator inserted = frontiers_.insert(frontiers_.end(), tmp_ftr);
      // Sort the viewpoints by info gain, best view in front
      sort(
          inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
          [](const Viewpoint& v1, const Viewpoint& v2) { return v1.info_gain_ > v2.info_gain_; });
      // make first new ftr point to newly inserted one (but only the first newly inserted)
      // Basically record the starting position of newly inserted one for future use
      if (first_new_ftr_ == frontiers_.end()) first_new_ftr_ = inserted;
    } else {
      // Find no viewpoint, move cluster to dormant list
      // tmp_ftr doesn't find good viewpoint
      dormant_frontiers_.push_back(tmp_ftr);
      ++new_dormant_num;
    }
  }
  // Reset indices of frontiers
  int idx = 0;
  for (auto& ft : frontiers_) {
    ft.id_ = idx++;
    std::cout << ft.id_ << ", ";
  }
  std::cout << "\nnew num: " << new_num << ", new dormant: " << new_dormant_num << std::endl;
  std::cout << "to visit: " << frontiers_.size() << ", dormant: " << dormant_frontiers_.size()
            << std::endl;
}


void FrontierFinder::getTopViewpointsIGNoPred(
  const Vector3d& cur_pos, vector<Eigen::Vector3d>& points, vector<double>& yaws,
  vector<Eigen::Vector3d>& averages, vector<int>& info_gains) {
  points.clear();
  yaws.clear();
  averages.clear();
  info_gains.clear();
  // for each of the frontier clusters, retrieve the viewpoint gives highest gain
  for (auto frontier : frontiers_) {
    bool no_view = true;
    for (auto view : frontier.viewpoints_) {
      // Retrieve the first viewpoint that is far enough and has highest coverage
      if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
      info_gains.push_back(view.info_gain_);
      no_view = false;
      break;
    }
    if (no_view) {
      // All viewpoints are very close, just use the first one (with highest coverage).
      auto view = frontier.viewpoints_.front();
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
      info_gains.push_back(view.info_gain_);
    }
  }
}

// TODO: accelerate this
void FrontierFinder::getCorrelationMatrix(const Vector3d& pos, const double& yaw, vector<vector<double>>& correlations) {
  ROS_WARN("Enter getCorrelationMatrix");
  correlations.clear();
  correlations.reserve(frontiers_.size()+1);
  vector<double> tmp_row(frontiers_.size()+1, 0);
  for (int i = 0; i < frontiers_.size()+1; i++) {
    correlations.push_back(tmp_row);
  }
  // for each of the frontier clusters, retrieve the viewpoint gives highest gain
  std::list<Frontier>::iterator it1, it2;
  int i = 1, j = 1;
  for (it1 = frontiers_.begin(); it1 != frontiers_.end(); it1++) {
    j = 1;
    for (it2 = frontiers_.begin(); it2 != frontiers_.end(); it2++) {
      if (it1->viewpoints_.size() == 0 || 
          it2->viewpoints_.size() == 0 || it1 == it2) {
        correlations[i][j] = 1;
      } else {
        if (i <= j) {
          correlations[i][j] = getNodeCorrelation(it1->viewpoints_[0], it2->viewpoints_[0]) * 0.1;
        }
        else {
          correlations[i][j] = correlations[j][i];
        }
      }
      j++;
    }
    i++;
  }
}

void FrontierFinder::getFrontiers(vector<vector<Eigen::Vector3d>>& clusters) {
  clusters.clear();
  for (auto frontier : frontiers_)
    clusters.push_back(frontier.cells_);
}


void FrontierFinder::getFrontierObjs(list<Frontier> &cluster_objs) {
  cluster_objs.clear();
  for (auto frontier : frontiers_)
    cluster_objs.push_back(frontier);
}


void FrontierFinder::getDormantFrontiers(vector<vector<Vector3d>>& clusters) {
  clusters.clear();
  for (auto ft : dormant_frontiers_)
    clusters.push_back(ft.cells_);
}

void FrontierFinder::getFrontierBoxes(vector<pair<Eigen::Vector3d, Eigen::Vector3d>>& boxes) {
  boxes.clear();
  for (auto frontier : frontiers_) {
    Vector3d center = (frontier.box_max_ + frontier.box_min_) * 0.5;
    Vector3d scale = frontier.box_max_ - frontier.box_min_;
    boxes.push_back(make_pair(center, scale));
  }
}


void FrontierFinder::copyFrontierToPlanning() {
  ROS_INFO_STREAM("Copy frontier to planning.");
  planning_frontiers_.clear();
  for (auto ftr: frontiers_) {
    Frontier ftr_copy;
    ftr_copy.average_ = ftr.average_;
    ftr_copy.id_ = ftr.id_;
    std::copy(ftr.viewpoints_.begin(), ftr.viewpoints_.end(), std::back_inserter(ftr_copy.viewpoints_));
    std::copy(ftr.costs_.begin(), ftr.costs_.end(), std::back_inserter(ftr_copy.costs_));
    std::copy(ftr.paths_.begin(), ftr.paths_.end(), std::back_inserter(ftr_copy.paths_));
    planning_frontiers_.push_back(ftr_copy);
  }
};

void FrontierFinder::getPathForTour(
    const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path) {
  // Make an frontier_indexer to access the frontier list easier
  vector<list<Frontier>::iterator> frontier_indexer;
  for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it)
    frontier_indexer.push_back(it);
  // Compute the path from current pos to the first frontier
  vector<Vector3d> segment;

  ViewNode::searchPath(pos, frontier_indexer[frontier_ids[0]]->viewpoints_.front().pos_, segment);
  path.insert(path.end(), segment.begin(), segment.end());
  // Get paths of tour passing all clusters
  for (int i = 0; i < frontier_ids.size() - 1; ++i) {
    // Move to path to next cluster
    auto path_iter = frontier_indexer[frontier_ids[i]]->paths_.begin();
    int next_idx = frontier_ids[i + 1];
    for (int j = 0; j < next_idx; ++j)
      ++path_iter;
    path.insert(path.end(), path_iter->begin(), path_iter->end());
  }
  path.insert(path.end(), pos);
}


void FrontierFinder::getPathForPlannedTour(
    const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path) {
  ROS_INFO("[getPathForPlannedTour] Start.");
  if (!global_tour_planned) {
    ROS_ERROR("[getPathForPlannedTour] Global tour not planned for this frontier set!");
    return;
  }
  ROS_INFO_STREAM("planning frontiers size:" << planning_frontiers_.size());
  ROS_INFO_STREAM("frontier_ids size:" << frontier_ids.size());
  // Make an frontier_indexer to access the frontier list easier
  vector<list<Frontier>::iterator> frontier_indexer;
  for (auto it = planning_frontiers_.begin(); it != planning_frontiers_.end(); ++it)
    frontier_indexer.push_back(it);
  // Compute the path from current pos to the first frontier
  vector<Vector3d> segment;

  ViewNode::searchPath(pos, frontier_indexer[frontier_ids[0]]->viewpoints_.front().pos_, segment);
  path.insert(path.end(), segment.begin(), segment.end());
  // Get paths of tour passing all clusters
  for (int i = 0; i < frontier_ids.size() - 1; ++i) {
    auto path_iter = frontier_indexer[frontier_ids[i]]->paths_.begin();
    int next_idx = frontier_ids[i + 1];
    for (int j = 0; j < next_idx; ++j) {
      ++path_iter;
    }

    path.insert(path.end(), path_iter->begin(), path_iter->end());
  }
  path.insert(path.end(), pos);
}


// This is only used to generate a global tour astar path. Typically for visualization purpose. 
void FrontierFinder::getPathForRefinedTour(
    const Vector3d& pos, const vector<std::string>& refined_sequence, const std::unordered_map<int, Eigen::Vector3d>& key_pose_pos, vector<Vector3d>& path) {
  ROS_INFO("[getPathForPlannedTour] Start.");
  if (!global_tour_planned) {
    ROS_ERROR("[getPathForPlannedTour] Global tour not planned for this frontier set!");
    return;
  }
  ROS_INFO_STREAM("[refined tour] planning frontiers size:" << planning_frontiers_.size());
  if (refined_sequence.size() == 0) {
    ROS_ERROR_STREAM("refined tour size:" << refined_sequence.size());
    return;
  }
  // Make an frontier_indexer to access the frontier list easier
  vector<list<Frontier>::iterator> frontier_indexer;
  for (auto it = planning_frontiers_.begin(); it != planning_frontiers_.end(); ++it)
    frontier_indexer.push_back(it);
  // Compute the path from current pos to the first frontier
  vector<Vector3d> segment;

  // Now we have two cases always, either to closure node or to next frontier
  if (refined_sequence[0].find("ftr") != std::string::npos) {
    ROS_INFO_STREAM("refined_sequence[0] is ftr" << refined_sequence[0]);
    // First node is a frontier
    int frontier_id = std::stoi(refined_sequence[0].substr(3));
    ViewNode::searchPath(pos, frontier_indexer[frontier_id]->viewpoints_.front().pos_, segment);
    path.insert(path.end(), segment.begin(), segment.end());
  } else {
    ROS_INFO_STREAM("refined_sequence[0] is clo" << refined_sequence[0]);
    // First node is a closure node
    int closure_id = std::stoi(refined_sequence[0].substr(3));
    ViewNode::searchPath(pos, key_pose_pos.at(closure_id), segment);
    path.insert(path.end(), segment.begin(), segment.end());
  }

  // Get paths of tour passing all clusters
  for (int i = 0; i < refined_sequence.size() - 1; ++i) {
    vector<Vector3d> tmp_segment;
    // Move to path to next cluster
    if (refined_sequence[i].find("ftr") != std::string::npos) {
      // First node is a frontier
      int frontier_id = std::stoi(refined_sequence[i].substr(3));
      auto path_iter = frontier_indexer[frontier_id]->paths_.begin();
      
      if (refined_sequence[i+1].find("ftr") != std::string::npos) {
        // Next node is a frontier
        int next_idx = std::stoi(refined_sequence[i+1].substr(3));
        for (int j = 0; j < next_idx; ++j) {
          // ROS_INFO_STREAM("what is this path size of this path iter point to " << path_iter->size());
          ++path_iter;
        }
        path.insert(path.end(), path_iter->begin(), path_iter->end());
      } else {
        // Next node is a closure node
        int closure_id = std::stoi(refined_sequence[i+1].substr(3));
        ViewNode::searchPath(frontier_indexer[frontier_id]->viewpoints_.front().pos_, key_pose_pos.at(closure_id), tmp_segment);
        path.insert(path.end(), tmp_segment.begin(), tmp_segment.end());
      }

    } else {
      // First node is a closure node
      int closure_id = std::stoi(refined_sequence[i].substr(3));
      if (refined_sequence[i+1].find("ftr") != std::string::npos) {
        // Next node is a frontier
        int frontier_id = std::stoi(refined_sequence[i+1].substr(3));
        ViewNode::searchPath(key_pose_pos.at(closure_id), frontier_indexer[frontier_id]->viewpoints_.front().pos_, tmp_segment);
        path.insert(path.end(), tmp_segment.begin(), tmp_segment.end());
      } else {
        // Next node is a closure node
        int next_closure_id = std::stoi(refined_sequence[i+1].substr(3));
        ViewNode::searchPath(key_pose_pos.at(closure_id), key_pose_pos.at(next_closure_id), tmp_segment);
        path.insert(path.end(), tmp_segment.begin(), tmp_segment.end());
      }
    }
  }
  // Go back to init position. 
  path.insert(path.end(), pos);
}


void FrontierFinder::getFullCostMatrix(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
    Eigen::MatrixXd& mat) {
  if (false) {
    // Use symmetric TSP formulation
    int dim = frontiers_.size() + 2;
    mat.resize(dim, dim);  // current pose (0), sites, and virtual depot finally

    int i = 1, j = 1;
    for (auto ftr : frontiers_) {
      for (auto cs : ftr.costs_)
        mat(i, j++) = cs;
      ++i;
      j = 1;
    }

    // Costs from current pose to sites
    for (auto ftr : frontiers_) {
      Viewpoint vj = ftr.viewpoints_.front();
      vector<Vector3d> path;
      mat(0, j) = mat(j, 0) =
          ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
      ++j;
    }
    // Costs from depot to sites, the same large vaule
    for (j = 1; j < dim - 1; ++j) {
      mat(dim - 1, j) = mat(j, dim - 1) = 100;
    }
    // Zero cost to depot to ensure connection
    mat(0, dim - 1) = mat(dim - 1, 0) = -10000;

  } else {
    ROS_INFO("Get Full cost mat");
    // Use Asymmetric TSP
    int dimen = frontiers_.size();
    mat.resize(dimen + 1, dimen + 1);
    if (debug_) {
      std::cout << "mat size: " << mat.rows() << ", " << mat.cols() << std::endl;
    }

    // Fill block for clusters
    int i = 1, j = 1;
    if (debug_) std::cout << "ftr size: " << frontiers_.size() << std::endl;

    for (auto ftr : frontiers_) {
      if (debug_) std::cout << "cost size: " << frontiers_.size() << std::endl;
      for (auto cs : ftr.costs_) {
        if (j > dimen) continue;
        if (debug_) std::cout << "(" << i << ", " << j << ")" << ", ";
        mat(i, j++) = cs;
      }
      ++i;
      j = 1;
    }
    if (debug_) std::cout << "mat is: \n" << mat << std::endl;
    if (debug_) std::cout << "" << std::endl;
    if (debug_) ROS_INFO("Fill first row and col");
    // Fill block from current state to clusters and also clusters to curr
    mat.leftCols<1>().setZero();
    for (auto ftr : frontiers_) {
      if (debug_) std::cout << "(0, " << j << ")" << ", ";
      Viewpoint vj = ftr.viewpoints_.front();
      vector<Vector3d> path;
      double tmp_cost = ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
      mat(0, j) = tmp_cost;
      mat(j, 0) = tmp_cost;
      j++;
    }
    if (debug_) std::cout << "" << std::endl;
    if (debug_) std::cout << "mat is: \n" << mat << std::endl;
  }
}

void FrontierFinder::findViewpoints(
    const Vector3d& sample, const Vector3d& ftr_avg, vector<Viewpoint>& vps) {
  if (!edt_env_->occ_map_->isInBox(sample) ||
      edt_env_->occ_map_->getInflateOccupancy(sample) == 1 || isNearUnknown(sample))
    return;

  double left_angle_, right_angle_, vertical_angle_, ray_length_;

  // Central yaw is determined by frontier's average position and sample
  auto dir = ftr_avg - sample;
  double hc = atan2(dir[1], dir[0]);

  vector<int> slice_gains;
  // Evaluate info gain of different slices
  for (double phi_h = -M_PI_2; phi_h <= M_PI_2 + 1e-3; phi_h += M_PI / 18) {
    // Compute gain of one slice
    int gain = 0;
    for (double phi_v = -vertical_angle_; phi_v <= vertical_angle_; phi_v += vertical_angle_ / 3) {
      // Find endpoint of a ray
      Vector3d end;
      end[0] = sample[0] + ray_length_ * cos(phi_v) * cos(hc + phi_h);
      end[1] = sample[1] + ray_length_ * cos(phi_v) * sin(hc + phi_h);
      end[2] = sample[2] + ray_length_ * sin(phi_v);

      // Do raycasting to check info gain
      Eigen::Vector3i idx;
      raycaster_->input(sample, end);
      while (raycaster_->nextId(idx)) {
        // Hit obstacle, stop the ray
        if (edt_env_->occ_map_->getOccupancy(idx) == 1 || !edt_env_->occ_map_->isInBox(idx))
          break;
        // Count number of unknown cells
        if (edt_env_->occ_map_->getOccupancy(idx) == SDFMap::UNKNOWN) ++gain;
      }
    }
    slice_gains.push_back(gain);
  }

  // Sum up slices' gain to get different yaw's gain
  vector<pair<double, int>> yaw_gains;
  for (int i = 0; i < 6; ++i)  // [-90,-10]-> [10,90], delta_yaw = 20, 6 groups
  {
    double yaw = hc - M_PI_2 + M_PI / 9.0 * i + right_angle_;
    int gain = 0;
    for (int j = 2 * i; j < 2 * i + 9; ++j)  // 80 degree hFOV, 9 slices
      gain += slice_gains[j];
    yaw_gains.push_back(make_pair(yaw, gain));
  }

  // Get several yaws with highest gain
  vps.clear();
  sort(
      yaw_gains.begin(), yaw_gains.end(),
      [](const pair<double, int>& p1, const pair<double, int>& p2) {
        return p1.second > p2.second;
      });
  for (int i = 0; i < 3; ++i) {
    if (yaw_gains[i].second < min_visib_num_) break;
    Viewpoint vp = { sample, yaw_gains[i].first, yaw_gains[i].second, 0};
    while (vp.yaw_ < -M_PI)
      vp.yaw_ += 2 * M_PI;
    while (vp.yaw_ > M_PI)
      vp.yaw_ -= 2 * M_PI;
    vps.push_back(vp);
  }
}


void FrontierFinder::sampleViewpointsIGNoPred(Frontier& frontier) {
  // Evaluate sample viewpoints on circles, find ones that give most information gain
  for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
       rc <= candidate_rmax_ + 1e-3; rc += dr) {
    for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_) {
      Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);
      if (sample_pos(2) < 0.9) {
        sample_pos(2) = 0.9;
      }
      // Qualified viewpoint is in bounding box and in safe region
      if (!edt_env_->occ_map_->isInBox(sample_pos) ||
          edt_env_->occ_map_->getInflateOccupancy(sample_pos) == 1 || isNearUnknown(sample_pos))
        continue;

      // Compute average yaw
      auto& cells = frontier.filtered_cells_;
      Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
      double avg_yaw = 0.0;
      for (int i = 1; i < cells.size(); ++i) {
        Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
        double yaw = acos(dir.dot(ref_dir));
        if (ref_dir.cross(dir)[2] < 0) yaw = -yaw;
        avg_yaw += yaw;
      }
      avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
      wrapYaw(avg_yaw);

      vector<Viewpoint> tmp_vps;
      // Compute information gain for sampled pos
      // currently return at most 3 yaw at each pos. return only best if not needed
      computeInfoGainNoPred(sample_pos, avg_yaw, tmp_vps);
      for (auto vp : tmp_vps) {
        frontier.viewpoints_.push_back(vp);
      }
    }
  }
}  

// in global map region
bool FrontierFinder::isFrontierCovered() {
  Vector3d update_min, update_max;
  //edt_env_->occ_map_->getUpdatedBox(update_min, update_max);

  auto checkChanges = [&](const list<Frontier>& frontiers) {
    for (auto ftr : frontiers) {
      //if (!haveOverlap(ftr.box_min_, ftr.box_max_, update_min, update_max)) continue;
      const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
      int change_num = 0;
      for (auto cell : ftr.cells_) {
        Eigen::Vector3i idx;
        edt_env_->occ_map_->posToIndex(cell, idx);
        if (!(knownfree(idx) && isNeighborUnknown(idx)) && ++change_num >= change_thresh)
          return true;
      }
    }
    return false;
  };

  if (checkChanges(frontiers_) || checkChanges(dormant_frontiers_)) return true;

  return false;
}


bool FrontierFinder::isNearUnknown(const Eigen::Vector3d& pos) {
  const int vox_num = floor(min_candidate_clearance_ / resolution_);
  for (int x = -vox_num; x <= vox_num; ++x)
    for (int y = -vox_num; y <= vox_num; ++y)
      for (int z = -1; z <= 1; ++z) {
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
        if (edt_env_->occ_map_->getOccupancy(vox) == SDFMap::UNKNOWN) return true;
      }
  return false;
}


bool FrontierFinder::isNearObstacle(const Eigen::Vector3d& pos) {
  const int vox_num = floor(min_candidate_obs_clearance_ / resolution_);
  for (int x = -vox_num; x <= vox_num; ++x)
    for (int y = -vox_num; y <= vox_num; ++y)
      for (int z = -4; z <= 4; ++z) {
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
        if (edt_env_->occ_map_->getOccupancy(vox) == SDFMap::OCCUPIED) return true;
      }
  return false;
}


int FrontierFinder::computeYawInfoGain(const Eigen::Vector3d& pos, 
                                       const double& yaw) 
{
  double half_v_fov, min_ray_length, max_ray_length, half_h_fov; // should be get from camera config
  half_v_fov = v_fov_/2;
  half_h_fov = h_fov_/2;
  max_ray_length = max_ray_length_;
  min_ray_length = min_ray_length_;
  // yaw is average yaw from current position to frontier clusters 

  int gain = 0;
  for (auto &frontier: frontiers_)
  {
    for (double phi_h = -half_h_fov; phi_h <= half_h_fov; phi_h += half_h_fov / 3) {
      for (double phi_v = -half_v_fov; phi_v <= half_v_fov; phi_v += half_v_fov / 3) {
        // Find endpoint of a ray
        Vector3d end, start;
        end[0] = pos[0] + max_ray_length * cos(phi_v) * cos(yaw + phi_h);
        end[1] = pos[1] + max_ray_length * cos(phi_v) * sin(yaw + phi_h);
        end[2] = pos[2] + max_ray_length * sin(phi_v);
        start[0] = pos[0] + min_ray_length * cos(phi_v) * cos(yaw + phi_h);
        start[1] = pos[1] + min_ray_length * cos(phi_v) * sin(yaw + phi_h);
        start[2] = pos[2] + min_ray_length * sin(phi_v);
        // TODO: this part
        // Do raycasting to check info gain
        Eigen::Vector3i idx;
        int local_idx;
        bool flag_unknown = false;
        raycaster_->input(start, end);
        while (raycaster_->nextId(idx)) {
          // Idx is global idx in map, check occupancy first
          // If hit obstacle, break
          if (edt_env_->occ_map_->getOccupancy(idx) == 1 || !edt_env_->occ_map_->isInBox(idx))
            break;
          // Cell is unknown in the original map
          if (edt_env_->occ_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
            gain ++;
          }
          // If cell is free, continue
        }
      }
    }

  }
  return gain;

}


// Given a sample pos, a frontier cluster yaw, return viewpoints provides best info gain
void FrontierFinder::computeInfoGainNoPred(const Eigen::Vector3d& pos, const double& yaw, vector<Viewpoint>& vps) {
  // TODO: these values should come from launch files. 
  double half_v_fov, min_ray_length, max_ray_length, half_h_fov; // should be get from camera config
  half_v_fov = v_fov_/2;
  half_h_fov = h_fov_/2;
  min_ray_length = min_ray_length_;
  max_ray_length = max_ray_length_;

  // yaw is average yaw from current position to frontier clusters 
  vector<int> slice_gains;
  // Evaluate info gain of different slices, horizontally 18 slices, 10 degree each
  for (double phi_h = -M_PI_2; phi_h <= M_PI_2 + 1e-3; phi_h += M_PI / 18) {
    // Compute gain of one slice at different height (vertical angle)
    int gain = 0;
    int potential_gain = 0;
    for (double phi_v = -half_v_fov; phi_v <= half_v_fov; phi_v += half_v_fov / 3) {
      // Find endpoint of a ray
      Vector3d end, start;
      end[0] = pos[0] + max_ray_length * cos(phi_v) * cos(yaw + phi_h);
      end[1] = pos[1] + max_ray_length * cos(phi_v) * sin(yaw + phi_h);
      end[2] = pos[2] + max_ray_length * sin(phi_v);
      start[0] = pos[0] + min_ray_length * cos(phi_v) * cos(yaw + phi_h);
      start[1] = pos[1] + min_ray_length * cos(phi_v) * sin(yaw + phi_h);
      start[2] = pos[2] + min_ray_length * sin(phi_v);

      // Do raycasting to check info gain
      Eigen::Vector3i idx;
      int local_idx;
      bool flag_unknown = false;
      raycaster_->input(start, end);  
      while (raycaster_->nextId(idx)) {
        // Idx is global idx in map, check occupancy first
        // If hit obstacle, break
        if (edt_env_->occ_map_->getOccupancy(idx) == 1 || !edt_env_->occ_map_->isInBox(idx))
          break;
        // Count number of unknown cells
        if (edt_env_->occ_map_->getOccupancy(idx) == SDFMap::UNKNOWN) ++gain;
      }
    }
    slice_gains.push_back(gain);
  }
  // Step 2: Sum up slices' gain to compute gain for each yaw of the sensor
  vector<pair<double, int>> yaw_gains;
  for (int i = 0; i < 6; ++i)  // [-90,-10]-> [10,90], delta_yaw = 20, 6 groups
  {
    double sensing_yaw = yaw - M_PI_2 + M_PI / 9.0 * i + half_h_fov;
    int gain = 0;
    for (int j = 2 * i; j < 2 * i + 9; ++j)  // 80 degree hFOV, 9 slices
      gain += slice_gains[j];
    yaw_gains.push_back(make_pair(sensing_yaw, gain));
  }
  // Step 3: find best yaw
  vps.clear();
  sort(
      yaw_gains.begin(), yaw_gains.end(),
      [](const pair<double, int>& p1, const pair<double, int>& p2) {
        return p1.second > p2.second;
      });
  for (int i = 0; i < 3; ++i) {
    if (yaw_gains[i].second < min_info_gain_) break;
    Viewpoint vp = {pos, yaw_gains[i].first, 0, yaw_gains[i].second };
    wrapYaw(vp.yaw_);
    vps.push_back(vp);
  }
}



void FrontierFinder::downsample(
    const vector<Eigen::Vector3d>& cluster_in, vector<Eigen::Vector3d>& cluster_out) {
  // downsamping cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto cell : cluster_in)
    cloud->points.emplace_back(cell[0], cell[1], cell[2]);

  // Downsample value=3 in config
  const double leaf_size = edt_env_->occ_map_->getResolution() * down_sample_;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloudf);

  cluster_out.clear();
  for (auto pt : cloudf->points)
    cluster_out.emplace_back(pt.x, pt.y, pt.z);
}

void FrontierFinder::wrapYaw(double& yaw) {
  while (yaw < -M_PI)
    yaw += 2 * M_PI;
  while (yaw > M_PI)
    yaw -= 2 * M_PI;
}

Eigen::Vector3i FrontierFinder::searchClearVoxel(const Eigen::Vector3i& pt) {
  queue<Eigen::Vector3i> init_que;
  vector<Eigen::Vector3i> nbrs;
  Eigen::Vector3i cur, start_idx;
  init_que.push(pt);
  // visited_flag_[toadr(pt)] = 1;

  while (!init_que.empty()) {
    cur = init_que.front();
    init_que.pop();
    if (knownfree(cur)) {
      start_idx = cur;
      break;
    }

    nbrs = sixNeighbors(cur);
    for (auto nbr : nbrs) {
      int adr = toadr(nbr);
    }
  }
  return start_idx;
}

inline vector<Eigen::Vector3i> FrontierFinder::sixNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(6);
  Eigen::Vector3i tmp;

  tmp = voxel - Eigen::Vector3i(1, 0, 0);
  neighbors[0] = tmp;
  tmp = voxel + Eigen::Vector3i(1, 0, 0);
  neighbors[1] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 1, 0);
  neighbors[2] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 1, 0);
  neighbors[3] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 0, 1);
  neighbors[4] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 0, 1);
  neighbors[5] = tmp;

  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::tenNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(10);
  Eigen::Vector3i tmp;
  int count = 0;

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0) continue;
      tmp = voxel + Eigen::Vector3i(x, y, 0);
      neighbors[count++] = tmp;
    }
  }
  neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(26);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0) continue;
        tmp = voxel + Eigen::Vector3i(x, y, z);
        neighbors[count++] = tmp;
      }
  return neighbors;
}

// here, should not use inflate occupancy
inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i& voxel) {
  // At least one neighbor is unknown
  auto nbrs = sixNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (edt_env_->occ_map_->getOccupancy(nbr) == SDFMap::UNKNOWN) return true;
  }
  return false;
}

inline int FrontierFinder::toadr(const Eigen::Vector3i& idx) {
  return edt_env_->occ_map_->toAddress(idx);
}

inline bool FrontierFinder::knownfree(const Eigen::Vector3i& idx) {
  return edt_env_->occ_map_->getOccupancy(idx) == SDFMap::FREE;
}

inline bool FrontierFinder::inmap(const Eigen::Vector3i& idx) {
  return edt_env_->occ_map_->isInMap(idx);
}



int FrontierFinder::occSearch(const Vector3d& pos) {
  return edt_env_->occ_map_->getOccupancy(pos);
}

double FrontierFinder::getNodeCorrelation(const Viewpoint& vp1, const Viewpoint& vp2) {
  vector<Point> vp1_pts, vp2_pts;
  Point vp1_pt1, vp1_pt2, vp1_pt3, vp2_pt1, vp2_pt2, vp2_pt3;
  double half_h_fov = h_fov_/2;
  double center_x, center_y;

  vp1_pt1.x_ = vp1.pos_(0);
  vp1_pt1.y_ = vp1.pos_(1);
  vp1_pt2.x_ = vp1.pos_(0) + max_ray_length_ * cos(0) * cos(vp1.yaw_ - half_h_fov);
  vp1_pt2.y_ = vp1.pos_(1) + max_ray_length_ * cos(0) * sin(vp1.yaw_ - half_h_fov);
  vp1_pt3.x_ = vp1.pos_(0) + max_ray_length_ * cos(0) * cos(vp1.yaw_ + half_h_fov);
  vp1_pt3.y_ = vp1.pos_(1) + max_ray_length_ * cos(0) * sin(vp1.yaw_ + half_h_fov);

  center_x = (vp1_pt1.x_ + vp1_pt2.x_ + vp1_pt3.x_) / 3;
  center_y = (vp1_pt1.y_ + vp1_pt2.y_ + vp1_pt3.y_) / 3;
  vp1_pt1.center_x_ = center_x;
  vp1_pt2.center_x_ = center_x;
  vp1_pt3.center_x_ = center_x;
  vp1_pt1.center_y_ = center_y;
  vp1_pt2.center_y_ = center_y;
  vp1_pt3.center_y_ = center_y;
  vp1_pts.push_back(vp1_pt1);
  vp1_pts.push_back(vp1_pt2);
  vp1_pts.push_back(vp1_pt3);

  // Second view frustum
  vp2_pt1.x_ = vp2.pos_(0);
  vp2_pt1.y_ = vp2.pos_(1);
  vp2_pt2.x_ = vp2.pos_(0) + max_ray_length_ * cos(0) * cos(vp2.yaw_ - half_h_fov);
  vp2_pt2.y_ = vp2.pos_(1) + max_ray_length_ * cos(0) * sin(vp2.yaw_ - half_h_fov);
  vp2_pt3.x_ = vp2.pos_(0) + max_ray_length_ * cos(0) * cos(vp2.yaw_ + half_h_fov);
  vp2_pt3.y_ = vp2.pos_(1) + max_ray_length_ * cos(0) * sin(vp2.yaw_ + half_h_fov);

  center_x = (vp2_pt1.x_ + vp2_pt2.x_ + vp2_pt3.x_) / 3;
  center_y = (vp2_pt1.y_ + vp2_pt2.y_ + vp2_pt3.y_) / 3;
  vp2_pt1.center_x_ = center_x;
  vp2_pt2.center_x_ = center_x;
  vp2_pt3.center_x_ = center_x;
  vp2_pt1.center_y_ = center_y;
  vp2_pt2.center_y_ = center_y;
  vp2_pt3.center_y_ = center_y;
  vp2_pts.push_back(vp2_pt1);
  vp2_pts.push_back(vp2_pt2);
  vp2_pts.push_back(vp2_pt3);
  // Sort both triangles coordinates in clockwise order
  sort(vp1_pts.begin(), vp1_pts.end(), clockwise_less);
  sort(vp2_pts.begin(), vp2_pts.end(), clockwise_less);  

  double vp1_points[20][2], vp2_points[3][2];
  vp1_points[0][0] = vp1_pts[0].x_;
  vp1_points[0][1] = vp1_pts[0].y_;
  vp1_points[1][0] = vp1_pts[1].x_;
  vp1_points[1][1] = vp1_pts[1].y_;  
  vp1_points[2][0] = vp1_pts[2].x_;
  vp1_points[2][1] = vp1_pts[2].y_;
  
  vp2_points[0][0] = vp2_pts[0].x_;
  vp2_points[0][1] = vp2_pts[0].y_;
  vp2_points[1][0] = vp2_pts[1].x_;
  vp2_points[1][1] = vp2_pts[1].y_;  
  vp2_points[2][0] = vp2_pts[2].x_;
  vp2_points[2][1] = vp2_pts[2].y_;

  // Find polygon intersection and polygon area
  double inter_area = suthHodgClip(vp1_points, 3, vp2_points, 3);
  if (inter_area != 0) {
    double X[] = {vp1_pt1.x_, vp1_pt2.x_, vp1_pt3.x_};
    double Y[] = {vp1_pt1.y_, vp1_pt2.y_, vp1_pt3.y_};
    double view_area = polygonArea(X, Y, 3);
    return inter_area/view_area;
  }
  return 0.0;
}


}  // namespace ms_planner