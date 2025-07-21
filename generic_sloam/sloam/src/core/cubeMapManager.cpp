#include <cubeMapManager.h>

CubeMapManager::CubeMapManager() {
  cube_landmarks_.reset(new CloudT);
}

std::vector<Cube> CubeMapManager::getVizMap() {
  std::vector<Cube> map;
  for (auto i = 0; i < cube_models_.size(); ++i) {
    // at least observed for how many times for the cuboid to be visualized
    if (cube_hits_[i] > 30) {
      map.push_back(cube_models_[i]);
    }
  }
  return map;
}

std::vector<Cube> &CubeMapManager::getMap() { return cube_models_; }

const std::vector<Cube> &CubeMapManager::getConstMap() const {
  return cube_models_;
}

std::map<int, int> CubeMapManager::getMatchesMap() const {
  return cubeMatchesMap_;
}

void CubeMapManager::getSubmap(const SE3 &pose,
                               std::vector<Cube> &cube_submap) {
  cube_submap.clear();

  if (cube_landmarks_->size() == 0) {
    std::cout << "no cube model yet, probably update map function has not been "
                 "called "
              << '\n';
    return;
  }

  cubeMatchesMap_.clear();
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cube_landmarks_);
  std::vector<int> pointIdxKNNSearch;
  std::vector<float> pointKNNSquaredDistance;
  PointT searchPoint;

  // Search for nearby cubes
  searchPoint.x = pose.translation()[0];
  searchPoint.y = pose.translation()[1];
  searchPoint.z = cube_landmarks_->points[0].z;

  int cube_match_sub_map_distance_threshold = 30;
  if (kdtree.nearestKSearch(searchPoint, cube_match_sub_map_distance_threshold,
                            pointIdxKNNSearch, pointKNNSquaredDistance) > 0) {
    int idx_count = 0;
    auto map_size = cube_models_.size();
    for (auto map_idx : pointIdxKNNSearch) {
      cubeMatchesMap_.insert(std::pair<int, int>(idx_count, map_idx));
      cube_submap.push_back(cube_models_[map_idx]);
      idx_count++;
    }
  } else {
    ROS_INFO("Not enough landmarks around pose: Total: %ld",
             pointIdxKNNSearch.size());
  }
}


// called in sloamNode.cpp
void CubeMapManager::updateMap(const SE3 &pose, std::vector<Cube> &obs_tms,
                               const std::vector<int> &cube_matches,
                               const int &robotID) {
  if (obs_tms.size() == 0) {
    std::cout << "no cubes are detected!" << '\n';
    return;
  }
  size_t i = 0;
  for (auto const &cube : obs_tms) {
    PointT pt;
    pt.x = static_cast<float>(cube.model.pose.translation()[0]);
    pt.y = static_cast<float>(cube.model.pose.translation()[1]);
    pt.z = static_cast<float>(cube.model.pose.translation()[2]);
    if (cube_matches[i] == -1) {
      cube_landmarks_->push_back(pt);
      cube_models_.push_back(cube);
      cube_hits_.push_back(1);
    } else {
      // transform from observation to map index
      int matchIdx = cubeMatchesMap_.at(cube_matches[i]);
      cube_hits_[matchIdx] += 1;
    }
    i++;
  }
}
