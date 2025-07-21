#pragma once

#include <cube.h>
#include <definitions.h>
#include <mapManager.h>

#include <set>

// MapManager in original SLOAM
class CubeMapManager {
 public:
  explicit CubeMapManager();
  // get map for visualization
  std::vector<Cube> getVizMap();
  const std::vector<Cube> &getConstMap() const;
  std::vector<Cube> &getMap();
  std::map<int, int> getMatchesMap() const;
  void getSubmap(const SE3 &pose, std::vector<Cube> &submap);

  void updateMap(const SE3 &pose, std::vector<Cube> &obs_tms,
                 const std::vector<int> &matches, const int &robotID);

 private:
  // landmarks_ variable only records the 3D positions of semantic landmarks for
  // purposes such as get sub map
  CloudT::Ptr cube_landmarks_;

  std::vector<size_t> cube_hits_;
  std::vector<Cube> cube_models_;

  std::map<int, int> cubeMatchesMap_;
};