#pragma once

#include <cylinder.h>
#include <definitions.h>
#include <plane.h>

#include <set>

struct KeyFrames {
  std::vector<SE3> poses;
  std::vector<std::vector<size_t>> landmark_idxs;
};

class MapManager {
 public:
  explicit MapManager(const float searchRadius = 50);
  std::vector<Cylinder> getVizMap();
  const std::vector<Cylinder> &getConstMap() const;
  std::vector<Cylinder> &getMap();
  std::map<int, int> getMatchesMap() const;
  void getSubmap(const SE3 &pose, std::vector<Cylinder> &submap);
  const std::vector<SE3> &getTrajectory(const int &robotID) const;

  void updateMap(const SE3 &pose, std::vector<Cylinder> &obs_tms,
                 const std::vector<int> &matches, const int &robotID);

 private:
  float sqSearchRadius;
  CloudT::Ptr landmarks_;
  CloudT::Ptr robot1PoseCloud;
  CloudT::Ptr robot2PoseCloud;

  std::vector<Cylinder> treeModels_;
  std::map<int, int> matchesMap_;
  std::vector<size_t> treeHits_;
  KeyFrames robot1KeyFrames_;
  KeyFrames robot2KeyFrames_;
};