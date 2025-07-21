#ifndef _SDF_MAP_H
#define _SDF_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <queue>
#include <ros/ros.h>
#include <tuple>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

namespace cv {
class Mat;
}

class RayCaster;

namespace ms_planner {
struct MapParam;
struct MapData;
class MapROS;

class SDFMap {
public:
  SDFMap();
  ~SDFMap();

  enum OCCUPANCY { UNKNOWN=-1, FREE=0, OCCUPIED=1};
  double logit(const double& x);

  void initMap(ros::NodeHandle& nh, shared_ptr<MapParam>& mp, shared_ptr<MapData>& md);
  void inputPointCloud(const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num,
                       const Eigen::Vector3d& camera_pos);

  void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  void posToIndexGivenMap(const Eigen::Vector3d& pos, Eigen::Vector3i& id, shared_ptr<MapParam>& mp);
  void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  void indexToPosGivenMap(const Eigen::Vector3i& id, Eigen::Vector3d& pos, shared_ptr<MapParam>& mp);

  void boundIndex(Eigen::Vector3i& id);
  void boundIndexGivenMap(Eigen::Vector3i& id, shared_ptr<MapParam>& mp);

  int toAddress(const Eigen::Vector3i& id);
  int toAddress(const int& x, const int& y, const int& z);
  int toAddressGivenMap(const int& x, const int& y, const int& z, shared_ptr<MapParam>& mp);

  bool isInMap(const Eigen::Vector3d& pos);
  bool isInMap(const Eigen::Vector3i& idx);
  bool isInGivenMap(const Eigen::Vector3i& idx, shared_ptr<MapParam>& mp);

  bool isInNoFlightZone(const Eigen::Vector3i& id);
  bool isInNoFlightZone(const Eigen::Vector3d& pos);

  bool isInBox(const Eigen::Vector3i& id);
  bool isInBox(const Eigen::Vector3d& pos);
  void boundBox(Eigen::Vector3d& low, Eigen::Vector3d& up);
  int getOccupancy(const Eigen::Vector3d& pos);
  int getOccupancy(const Eigen::Vector3i& id);
  double getOriOccupancy(const Eigen::Vector3d& pos);
  double getOriOccupancy(const Eigen::Vector3i& id);
  void setOccupied(const Eigen::Vector3d& pos, const int& occ = 1);
  int getInflateOccupancy(const Eigen::Vector3d& pos);
  int getInflateOccupancy(const Eigen::Vector3i& id);
  double getDistance(const Eigen::Vector3d& pos);
  double getDistance(const Eigen::Vector3i& id);
  double getDistWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);
  void resetBuffer();
  void resetBuffer(const Eigen::Vector3d& min, const Eigen::Vector3d& max);

  void genPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, const double& vis_z_low, const double& vis_z_high, const string& frame_id);
  void genInflatePointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, const double& vis_z_low, const double& vis_z_high, const string& frame_id);
  void genInflateLocalPointCloud(shared_ptr<MapParam>& mp, shared_ptr<MapData>& md, const Eigen::Vector3d& offset, pcl::PointCloud<pcl::PointXYZ>& cloud, const double& vis_z_low, const double& vis_z_high, const string& frame_id);

  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
  void getBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax);
  void getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax, bool reset = false);
  double getResolution();
  int getVoxelNum();
  int getMapDefault();

  void initInflateRegion();
  shared_ptr<MapParam> mp_;

  // void getPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud);
  void getLocalPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);
  void getAllPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);


  void getInflatedLocalMap(shared_ptr<MapParam>& mp, shared_ptr<MapData>& md);
  void decayLocalCloud(const Eigen::Vector3d& pos, double max_decay_range);

private:
  void clearAndInflateLocalMap();
  void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);
  void setCacheOccupancy(const Eigen::Vector3i& idx, const int& adr, const int& occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

  std::vector<Eigen::Vector3i> local_infla_array_;

  shared_ptr<MapData> md_;
  // unique_ptr<MapROS> mr_;
  unique_ptr<RayCaster> caster_;
  Eigen::Vector3i curr_offset_;

  friend MapROS;
  
  //for corridor generations
  pcl::PointCloud<pcl::PointXYZ> latest_cloud_all_, latest_cloud_local_;
  double val_decay_per_sec_;
  ros::Time last_decay_time_;
  bool first_decay_;
public:
  typedef std::shared_ptr<SDFMap> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct MapParam {
  // map properties
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;
  Eigen::Vector3i map_voxel_num_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  double inflation_xy_, inflation_z_; 
  double virtual_ceil_height_, ground_height_;
  Eigen::Vector3i box_min_, box_max_;
  Eigen::Vector3d box_mind_, box_maxd_;
  double default_dist_;
  bool optimistic_, signed_dist_;
  // map fusion
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_;  // logit
  double max_ray_length_;
  double map_update_bound_inflate_;
  bool inflate_local_;
  int local_map_margin_;
  double unknown_flag_;
  double decay_times_to_empty_;
  double decay_amount_lower_than_occ_;
  double decay_amount_lower_than_occ_log_;
  // No flight zone
  bool set_no_flight_zone_;
  int nfz_num_;
  std::vector<Eigen::Vector3d> nfz_min_;
  std::vector<Eigen::Vector3d> nfz_max_;
  std::vector<Eigen::Vector3i> nfz_min_id_;
  std::vector<Eigen::Vector3i> nfz_max_id_;

};

struct MapData {
  // main map data, occupancy of each voxel and Euclidean distance
  std::vector<double> occupancy_buffer_;
  std::vector<double> occupancy_buffer_inflate_;
  // data for updating
  vector<short> count_hit_, count_miss_, count_hit_and_miss_;
  vector<char> flag_rayend_, flag_visited_;
  char raycast_num_;
  queue<int> cache_voxel_;
  queue<Eigen::Vector3i> cache_voxel_idx_;
  Eigen::Vector3i local_bound_min_, local_bound_max_;
  Eigen::Vector3d update_min_, update_max_;
  bool reset_updated_box_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Pos to index is not a safe operation!!!!!!!
// ID may out of boundary
// Call BoundIndex() to bound the index
inline void SDFMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) {
    id(i) = floor((pos(i) - mp_->map_origin_(i)) * mp_->resolution_inv_);
    id(i) = max(min(id(i), mp_->map_voxel_num_(i) - 1), 0);
  }
}

// Pos to index given the map
inline void SDFMap::posToIndexGivenMap(const Eigen::Vector3d& pos, Eigen::Vector3i& id, shared_ptr<MapParam>& mp) {
  for (int i = 0; i < 3; ++i) {
    id(i) = floor((pos(i) - mp->map_origin_(i)) * mp->resolution_inv_);
    id(i) = max(min(id(i), mp->map_voxel_num_(i) - 1), 0);
  }
}

inline void SDFMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mp_->resolution_ + mp_->map_origin_(i);
}

inline void SDFMap::indexToPosGivenMap(const Eigen::Vector3i& id, Eigen::Vector3d& pos, shared_ptr<MapParam>& mp) {
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mp->resolution_ + mp->map_origin_(i);
}

inline void SDFMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_->map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_->map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_->map_voxel_num_(2) - 1), 0);
  id = id1;
}

// bound index given the map
inline void SDFMap::boundIndexGivenMap(Eigen::Vector3i& id, shared_ptr<MapParam>& mp) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp->map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp->map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp->map_voxel_num_(2) - 1), 0);
  id = id1;
}

inline int SDFMap::toAddress(const int& x, const int& y, const int& z) {
  return x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) + y * mp_->map_voxel_num_(2) + z;
}

inline int SDFMap::toAddress(const Eigen::Vector3i& id) {
  return toAddress(id[0], id[1], id[2]);
}

inline int SDFMap::toAddressGivenMap(const int& x, const int& y, const int& z, shared_ptr<MapParam>& mp) {
  return x * mp->map_voxel_num_(1) * mp->map_voxel_num_(2) + y * mp->map_voxel_num_(2) + z;
}


inline bool SDFMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_->map_min_boundary_(0) + 1e-4 || pos(1) < mp_->map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_->map_min_boundary_(2) + 1e-4)
    return false;
  if (pos(0) > mp_->map_max_boundary_(0) - 1e-4 || pos(1) > mp_->map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_->map_max_boundary_(2) - 1e-4)
    return false;
  return true;
}

inline bool SDFMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) return false;
  if (idx(0) > mp_->map_voxel_num_(0) - 1 || idx(1) > mp_->map_voxel_num_(1) - 1 ||
      idx(2) > mp_->map_voxel_num_(2) - 1)
    return false;
  return true;
}

inline bool SDFMap::isInGivenMap(const Eigen::Vector3i& idx, shared_ptr<MapParam>& mp) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) return false;
  if (idx(0) > mp->map_voxel_num_(0) - 1 || idx(1) > mp->map_voxel_num_(1) - 1 ||
      idx(2) > mp->map_voxel_num_(2) - 1)
    return false;
  return true;
}

inline bool SDFMap::isInBox(const Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) {
    if (id[i] < mp_->box_min_[i] || id[i] >= mp_->box_max_[i]) {
      return false;
    }
  }
  return true;
}

inline bool SDFMap::isInBox(const Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) {
    if (pos[i] <= mp_->box_mind_[i] || pos[i] >= mp_->box_maxd_[i]) {
      return false;
    }
  }
  return true;
}

inline bool SDFMap::isInNoFlightZone(const Eigen::Vector3i& id) {
  if (! mp_->set_no_flight_zone_) return false;
  int no_flight_zone_counter = 0;
  for (int i = 0; i < mp_->nfz_num_; i++) {
    for (int j = 0; j < 3; ++j) {
      if (id[j] <= mp_->nfz_min_id_[i][j] || id[j] >= mp_->nfz_max_id_[i][j]) {
        no_flight_zone_counter ++;
        break;
      }
    }
  }
  if (no_flight_zone_counter == mp_->nfz_num_) return false;
  return true;
}

inline bool SDFMap::isInNoFlightZone(const Eigen::Vector3d& pos) {
  if (! mp_->set_no_flight_zone_) return false;
  int no_flight_zone_counter = 0;
  for (int i = 0; i < mp_->nfz_num_; i++) {
    for (int j = 0; j < 3; ++j) {
      if (pos[j] <= mp_->nfz_min_[i][j] || pos[j] >= mp_->nfz_max_[i][j]) {
        no_flight_zone_counter ++;
        break;
      }
    }
  }
  if (no_flight_zone_counter == mp_->nfz_num_) return false;
  return true;
}

inline void SDFMap::boundBox(Eigen::Vector3d& low, Eigen::Vector3d& up) {
  for (int i = 0; i < 3; ++i) {
    low[i] = max(low[i], mp_->box_mind_[i]);
    up[i] = min(up[i], mp_->box_maxd_[i]);
  }
}

inline int SDFMap::getOccupancy(const Eigen::Vector3i& id) {
  if (!isInMap(id)) return -1;
  double occ = md_->occupancy_buffer_[toAddress(id)];
  if (occ < mp_->clamp_min_log_ - 1e-3) return UNKNOWN;
  if (occ > mp_->min_occupancy_log_) return OCCUPIED;
  return FREE;
}

inline int SDFMap::getOccupancy(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getOccupancy(id);
}

inline double SDFMap::getOriOccupancy(const Eigen::Vector3i& id) {
  if (!isInMap(id)) return -1;
  double occ = md_->occupancy_buffer_[toAddress(id)];
  return occ;
}

inline double SDFMap::getOriOccupancy(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getOriOccupancy(id);
}

inline void SDFMap::setOccupied(const Eigen::Vector3d& pos, const int& occ) {
  if (!isInMap(pos)) return;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  md_->occupancy_buffer_inflate_[toAddress(id)] = occ;
}

inline int SDFMap::getInflateOccupancy(const Eigen::Vector3i& id) {
  if (!isInMap(id)) return -1;
  // return int(md_->occupancy_buffer_inflate_[toAddress(id)]);
  double occ = md_->occupancy_buffer_inflate_[toAddress(id)];
  if (occ < mp_->clamp_min_log_ - 1e-3) return UNKNOWN;
  if (occ > mp_->min_occupancy_log_) return OCCUPIED;
  return FREE;
}

inline int SDFMap::getInflateOccupancy(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getInflateOccupancy(id);
}


inline void SDFMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {
  int num = 0;

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z) {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
      }
}
}
#endif