#include <mapManager.h>

MapManager::MapManager(const float searchRadius) {
  sqSearchRadius = searchRadius * searchRadius;
  landmarks_.reset(new CloudT);
  robot1PoseCloud.reset(new CloudT);
  robot2PoseCloud.reset(new CloudT);
}

void MapManager::updateMap(const SE3 &pose, std::vector<Cylinder> &obs_tms,
                           const std::vector<int> &matches,
                           const int &robotID) {
  if (obs_tms.size() == 0) return;

  size_t i = 0;
  std::vector<size_t> lidxs;
  for (auto const &tree : obs_tms) {
    PointT pt;
    pt.x = tree.model.root[0];
    pt.y = tree.model.root[1];
    pt.z = tree.model.root[2];
    if (matches[i] == -1) {
      // use treeModels.size() as its index since it is a new landmark
      lidxs.push_back(treeModels_.size());
      landmarks_->push_back(pt);
      treeModels_.push_back(tree);
      treeHits_.push_back(1);
    } else {
      // transform from observation to map index
      int matchIdx = matchesMap_.at(matches[i]);
      treeHits_[matchIdx] += 1;
      lidxs.push_back(matchIdx);
    }
    i++;
  }

  PointT posePt;
  posePt.x = float(pose.translation()[0]);
  posePt.y = float(pose.translation()[1]);
  posePt.z = float(pose.translation()[2]);

  if (robotID == 0) {
    robot1PoseCloud->points.push_back(posePt);
    robot1KeyFrames_.poses.push_back(pose);
  } else if (robotID == 1) {
    robot2PoseCloud->points.push_back(posePt);
    robot2KeyFrames_.poses.push_back(pose);
  }
}

std::vector<Cylinder> MapManager::getVizMap() {
  std::vector<Cylinder> map;
  for (auto i = 0; i < treeModels_.size(); ++i) {
    if (treeHits_[i] > 0) map.push_back(treeModels_[i]);
  }
  return map;
}

std::vector<Cylinder> &MapManager::getMap() { return treeModels_; }

const std::vector<Cylinder> &MapManager::getConstMap() const {
  return treeModels_;
}

std::map<int, int> MapManager::getMatchesMap() const { return matchesMap_; }

const std::vector<SE3> &MapManager::getTrajectory(const int &robotID) const {
  if (robotID == 0) {
    return robot1KeyFrames_.poses;
  } else if (robotID == 1) {
    return robot2KeyFrames_.poses;
  } else {
    printf("############# Error: invalid robotID!!! #############\n");
    return robot1KeyFrames_.poses;
  }
}


void MapManager::getSubmap(const SE3 &pose, std::vector<Cylinder> &submap) {
  if (landmarks_->size() == 0) return;

  matchesMap_.clear();
  // should already be empty but just to make sure
  submap.clear();
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(landmarks_);
  std::vector<int> pointIdxKNNSearch;
  std::vector<float> pointKNNSquaredDistance;
  PointT searchPoint;

  // Search for nearby trees
  searchPoint.x = pose.translation()[0];
  searchPoint.y = pose.translation()[1];
  searchPoint.z = 1;
  if (kdtree.nearestKSearch(searchPoint, 100, pointIdxKNNSearch,
                            pointKNNSquaredDistance) > 0) {
    int idx_count = 0;
    auto map_size = treeModels_.size();
    for (auto map_idx : pointIdxKNNSearch) {
      matchesMap_.insert(std::pair<int, int>(idx_count, map_idx));
      submap.push_back(treeModels_[map_idx]);
      idx_count++;
    }
  } else {
    ROS_INFO("Not enough landmarks around pose: Total: %ld",
             pointIdxKNNSearch.size());
  }
}