#include "plan_env/sdf_map.h"
#include "plan_env/map_ros.h"
#include <plan_env/raycast.h>

namespace ms_planner {
SDFMap::SDFMap() {
}

SDFMap::~SDFMap() {
}

void SDFMap::initMap(ros::NodeHandle& nh, shared_ptr<MapParam>& mp, shared_ptr<MapData>& md) {
  mp_ = mp;
  md_ = md;

  mp_->map_update_bound_inflate_ = max(mp_->resolution_, mp_->map_update_bound_inflate_);
  mp_->resolution_inv_ = 1 / mp_->resolution_;
  for (int i = 0; i < 3; ++i) {
    mp_->map_voxel_num_(i) = ceil(mp_->map_size_(i) / mp_->resolution_);
    ROS_INFO_STREAM("Map voxel num: " << mp_->map_voxel_num_(i));
  }
  mp_->prob_hit_log_ = logit(mp_->p_hit_);
  mp_->prob_miss_log_ = logit(mp_->p_miss_);
  mp_->clamp_min_log_ = logit(mp_->p_min_);
  mp_->clamp_max_log_ = logit(mp_->p_max_);
  mp_->min_occupancy_log_ = logit(mp_->p_occ_);
  mp_->decay_amount_lower_than_occ_log_ = logit(mp_->decay_amount_lower_than_occ_);

  mp_->unknown_flag_ = 0.01;


  // Initialize data buffer of map
  int buffer_size = mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
  md_->occupancy_buffer_ = vector<double>(buffer_size, mp_->clamp_min_log_ - mp_->unknown_flag_);
  md_->occupancy_buffer_inflate_ = vector<double>(buffer_size, mp_->clamp_min_log_ - mp_->unknown_flag_);
  md_->count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_->count_hit_ = vector<short>(buffer_size, 0);
  md_->count_miss_ = vector<short>(buffer_size, 0);
  md_->flag_rayend_ = vector<char>(buffer_size, -1);
  md_->flag_visited_ = vector<char>(buffer_size, -1);
  md_->raycast_num_ = 0;
  md_->reset_updated_box_ = true;
  md_->update_min_ = md_->update_max_ = Eigen::Vector3d(0, 0, 0);
  cout << "hit: " << mp_->prob_hit_log_ << ", miss: " << mp_->prob_miss_log_
       << ", min: " << mp_->clamp_min_log_ << ", max: " << mp_->clamp_max_log_
       << ", thresh: " << mp_->min_occupancy_log_ << endl;

  posToIndex(mp_->box_mind_, mp_->box_min_);
  ROS_INFO_STREAM("Box min d: " << mp_->box_mind_.transpose() << "  Box min: " << mp_->box_min_.transpose());
  posToIndex(mp_->box_maxd_, mp_->box_max_);
  ROS_INFO_STREAM("Box max d: " << mp_->box_maxd_.transpose() << "  Box max: " << mp_->box_max_.transpose());

  // Compute No Flight Zone ID
  if (mp_->set_no_flight_zone_) {
    for (int i = 0; i < mp_->nfz_num_; i++) 
    {
      Eigen::Vector3i tmp_id_min, tmp_id_max;
      posToIndex(mp_->nfz_min_[i], tmp_id_min);
      posToIndex(mp_->nfz_max_[i], tmp_id_max);
      mp_->nfz_min_id_.push_back(tmp_id_min);
      mp_->nfz_max_id_.push_back(tmp_id_max);
    }

    ROS_INFO("Set no flight zone to be occupied.");
    // Init no flight zone as occupied.
    for (int i = 0; i < mp_->nfz_num_; i++) 
    {
      for (int x = mp_->nfz_min_id_[i](0); x <= mp_->nfz_max_id_[i](0); x++) 
      {
        for (int y = mp_->nfz_min_id_[i](1); y <= mp_->nfz_max_id_[i](1); y++) 
        {
          for (int z = mp_->nfz_min_id_[i](2); z <= mp_->nfz_max_id_[i](2); z++) 
          {
            int adr = toAddress(x, y, z);
            md_->occupancy_buffer_[adr] = mp_->clamp_max_log_;
            md_->occupancy_buffer_inflate_[adr] = mp_->clamp_max_log_;
          }
        }
      }
    }
  }
  
  // Init inflate array
  initInflateRegion();

  // Init map decay value

  if (mp_->decay_times_to_empty_ >= 1) {
    val_decay_per_sec_ = (mp_->clamp_max_log_ - (mp_->min_occupancy_log_ - mp_->decay_amount_lower_than_occ_log_)) / mp_->decay_times_to_empty_;
  } else {
    val_decay_per_sec_ = 0;  // no decay
  }
  first_decay_ = true;

  caster_.reset(new RayCaster);
  caster_->setParams(mp_->resolution_, mp_->map_origin_);
}

double SDFMap::logit(const double& x) {
  return log(x / (1 - x));
}

void SDFMap::resetBuffer() {
  resetBuffer(mp_->map_min_boundary_, mp_->map_max_boundary_);
  md_->local_bound_min_ = Eigen::Vector3i::Zero();
  md_->local_bound_max_ = mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
}

void SDFMap::resetBuffer(const Eigen::Vector3d& min_pos, const Eigen::Vector3d& max_pos) {
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);


  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        //md_->distance_buffer_[toAddress(x, y, z)] = mp_->default_dist_;
      }
}

void SDFMap::initInflateRegion() {
  const double res = mp_->resolution_;
  local_infla_array_.clear();
  int rn = std::ceil(mp_->inflation_xy_ / res);
  int hn = std::ceil(mp_->inflation_z_ / res);
  for (int nx = -rn; nx <= rn; ++nx) {
    for (int ny = -rn; ny <= rn; ++ny) {
      for (int nz = -hn; nz <= hn; ++nz) {
        if (nx == 0 && ny == 0) continue;
        if (std::hypot(nx, ny) > rn) continue;
        local_infla_array_.push_back(Eigen::Vector3i(nx, ny, nz));
      }
    }
  }
}

template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[mp_->map_voxel_num_(dim)];
  double z[mp_->map_voxel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q)
      k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}


void SDFMap::setCacheOccupancy(const Eigen::Vector3i& idx, const int& adr, const int& occ) {
  // Add to update list if first visited
  if (md_->count_hit_[adr] == 0 && md_->count_miss_[adr] == 0) {
    md_->cache_voxel_.push(adr);
    md_->cache_voxel_idx_.push(idx);
  }

  if (occ == 0)
    md_->count_miss_[adr] = 1;
  else if (occ == 1)
    md_->count_hit_[adr] += 1;
}

void SDFMap::genPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, const double& vis_z_low, const double& vis_z_high, const string& frame_id) {
  pcl::PointXYZ pt;
  for (int x = mp_->box_min_(0) /* + 1 */; x < mp_->box_max_(0); ++x)
  for (int y = mp_->box_min_(1) /* + 1 */; y < mp_->box_max_(1); ++y)
    for (int z = mp_->box_min_(2) /* + 1 */; z < mp_->box_max_(2); ++z) {
      if (md_->occupancy_buffer_[toAddress(x, y, z)] > mp_->min_occupancy_log_)
      {
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > vis_z_high) continue;
        if (pos(2) < vis_z_low) continue;
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }
    }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id;
}

void SDFMap::genInflatePointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, const double& vis_z_low, const double& vis_z_high, const string& frame_id) {
  pcl::PointXYZ pt;
  for (int x = mp_->box_min_(0) /* + 1 */; x < mp_->box_max_(0); ++x)
    for (int y = mp_->box_min_(1) /* + 1 */; y < mp_->box_max_(1); ++y)
      for (int z = mp_->box_min_(2) /* + 1 */; z < mp_->box_max_(2); ++z) {
        if (md_->occupancy_buffer_inflate_[toAddress(x, y, z)] >= mp_->min_occupancy_log_)
        {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > vis_z_high) continue;
          if (pos(2) < vis_z_low) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id;
  latest_cloud_all_ = cloud;
}

void SDFMap::genInflateLocalPointCloud(shared_ptr<MapParam>& mp, shared_ptr<MapData>& md, const Eigen::Vector3d& offset, pcl::PointCloud<pcl::PointXYZ>& cloud, const double& vis_z_low, const double& vis_z_high, const string& frame_id) {
  pcl::PointXYZ pt;
  for (int x = md->local_bound_min_(0) /* + 1 */; x < md->local_bound_max_(0); ++x)
    for (int y = md->local_bound_min_(1) /* + 1 */; y < md->local_bound_max_(1); ++y)
      for (int z = md->local_bound_min_(2) /* + 1 */; z < md->local_bound_max_(2); ++z) {
        if (md->occupancy_buffer_inflate_[toAddressGivenMap(x, y, z, mp)] >= mp_->min_occupancy_log_)
        {
          Eigen::Vector3d pos;
          // Now we publish this map into vio frame, so we need to compensate the offset
          // The origin of local map is keep changing, the local map will vibrate
          // indexToPosGivenMap(Eigen::Vector3i(x, y, z), pos, mp);
          // Try to use fixed origin which is storage map
          indexToPos(Eigen::Vector3i(x+curr_offset_(0), y+curr_offset_(1), z+curr_offset_(2)), pos);
          if (pos(2) > vis_z_high) continue;
          if (pos(2) < vis_z_low) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id;
  latest_cloud_local_ = cloud;
}


void SDFMap::getLocalPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud){
  cloud = latest_cloud_local_;
}

void SDFMap::getAllPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud){

  cloud = latest_cloud_all_;
  // mr_->getPointCloud(cloud);
}


// Crop the local map from storage map with the given map param and map data
void SDFMap::getInflatedLocalMap(shared_ptr<MapParam>& mp, shared_ptr<MapData>& md) {
  posToIndexGivenMap(mp->map_origin_+mp->map_size_, md->local_bound_max_, mp);
  posToIndexGivenMap(mp->map_origin_, md->local_bound_min_, mp);

  for (int i = 0; i < 3; ++i) {
    curr_offset_(i) = floor((mp->map_origin_(i) - mp_->map_origin_(i)) * mp->resolution_inv_);
  }
  // Iterate through all voxels in local_bound_min to local_bound_max
  for (int x = md->local_bound_min_(0); x <= md->local_bound_max_(0); ++x) {
    for (int y = md->local_bound_min_(1); y <= md->local_bound_max_(1); ++y) {
      for (int z = md->local_bound_min_(2); z <= md->local_bound_max_(2); ++z) {
        Eigen::Vector3i id_storage(x+curr_offset_(0), y+curr_offset_(1), z+curr_offset_(2));
        int idx_local_map = toAddressGivenMap(x, y, z, mp);
        int idx_storage_map = toAddress(x+curr_offset_(0), y+curr_offset_(1), z+curr_offset_(2));
        if (!isInMap(id_storage)) {
          // Given address it out of storage map, treat as obstacle
          md->occupancy_buffer_inflate_[idx_local_map] = mp->min_occupancy_log_;
        } else {
          md->occupancy_buffer_inflate_[idx_local_map] = md_->occupancy_buffer_inflate_[idx_storage_map];
        }
      }
    }
  }
}



void SDFMap::inputPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num,
    const Eigen::Vector3d& camera_pos) {
  if (point_num == 0) return;
  // Safety check if camera_pos is inside map boundary
  if (!isInMap(camera_pos)) {
    ROS_ERROR("Camera position is outside map boundary!");
    return;
  }
  // Decay cloud which is within a local region around the robot
  if (val_decay_per_sec_ > 0) {
    if (first_decay_) {
      first_decay_ = false;
      last_decay_time_ = ros::Time::now();
    } else {
      double max_decay_range = mp_->max_ray_length_ * 2.0;
      decayLocalCloud(camera_pos, max_decay_range);
      last_decay_time_ = ros::Time::now();
    }
  }
  md_->raycast_num_ += 1;

  Eigen::Vector3d update_min = camera_pos;
  Eigen::Vector3d update_max = camera_pos;
  if (md_->reset_updated_box_) {
    md_->update_min_ = camera_pos;
    md_->update_max_ = camera_pos;
    md_->reset_updated_box_ = false;
  }

  Eigen::Vector3d pt_w, tmp;
  Eigen::Vector3i idx;
  int vox_adr;
  double length;
  for (int i = 0; i < point_num; ++i) {
    auto& pt = points.points[i];
    pt_w << pt.x, pt.y, pt.z;
    int tmp_flag;
    // Set flag for projected point
    if (!isInMap(pt_w)) {
      // Find closest point in map and set free
      pt_w = closetPointInMap(pt_w, camera_pos);
      length = (pt_w - camera_pos).norm();
      if (length > mp_->max_ray_length_)
        pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
      if (pt_w[2] < 0.2) continue;
      tmp_flag = 0;
    } else {
      length = (pt_w - camera_pos).norm();
      if (length > mp_->max_ray_length_) {
        pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
        if (pt_w[2] < 0.2) continue;
        tmp_flag = 0;
      } else
        tmp_flag = 1;
    }
    posToIndex(pt_w, idx);
    vox_adr = toAddress(idx);
    setCacheOccupancy(idx, vox_adr, tmp_flag);

    for (int k = 0; k < 3; ++k) {
      update_min[k] = min(update_min[k], pt_w[k]);
      update_max[k] = max(update_max[k], pt_w[k]);
    }
    // Raycasting between camera center and point
    if (md_->flag_rayend_[vox_adr] == md_->raycast_num_)
      continue;
    else
      md_->flag_rayend_[vox_adr] = md_->raycast_num_;

    caster_->input(pt_w, camera_pos);
    caster_->nextId(idx);
    while (caster_->nextId(idx)) {
      setCacheOccupancy(idx, toAddress(idx), 0);
    }
  }

  Eigen::Vector3d bound_inf(mp_->map_update_bound_inflate_, mp_->map_update_bound_inflate_, 0);
  posToIndex(update_max + bound_inf, md_->local_bound_max_);
  posToIndex(update_min - bound_inf, md_->local_bound_min_);

  // Bounding box for subsequent updating
  for (int k = 0; k < 3; ++k) {
    md_->update_min_[k] = min(update_min[k], md_->update_min_[k]);
    md_->update_max_[k] = max(update_max[k], md_->update_max_[k]);
  }

  auto t1 = ros::Time::now();
  while (!md_->cache_voxel_.empty()) {
    int adr = md_->cache_voxel_.front();
    md_->cache_voxel_.pop();
    // idx is used to update global inflate map
    Eigen::Vector3i tmp_idx = md_->cache_voxel_idx_.front();
    md_->cache_voxel_idx_.pop();

    // Use idx to check if in no flight zone, if yes, skip the following process:
    if (mp_->set_no_flight_zone_) {
      if (isInNoFlightZone(tmp_idx)) {
        continue;
      }
    }

    double log_odds_update =
        md_->count_hit_[adr] >= md_->count_miss_[adr] ? mp_->prob_hit_log_ : mp_->prob_miss_log_;
    md_->count_hit_[adr] = md_->count_miss_[adr] = 0;
    // If unknown, make it known
    if (md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3)
      md_->occupancy_buffer_[adr] = mp_->clamp_min_log_;
    // Bound between min and max
    md_->occupancy_buffer_[adr] = std::min(
        std::max(md_->occupancy_buffer_[adr] + log_odds_update, mp_->clamp_min_log_),
        mp_->clamp_max_log_);
    

    // 1. Do the same to voxels in the inflation region
    // If unknown, make it known
    if (md_->occupancy_buffer_inflate_[adr] < mp_->clamp_min_log_ - 1e-3)
      md_->occupancy_buffer_inflate_[adr] = mp_->clamp_min_log_;
    // Bound between min and max
    md_->occupancy_buffer_inflate_[adr] = std::min(
        std::max(md_->occupancy_buffer_inflate_[adr] + log_odds_update, mp_->clamp_min_log_),
        mp_->clamp_max_log_);

    // 2. Inflate only occupied:
    // Note: in the inflated map, the inflated occupied cell won't be cleared. 
    
    if (getOccupancy(tmp_idx) == OCCUPIED) {
      for (const auto& it_n: local_infla_array_) {
        Eigen::Vector3i neighbor_idx = tmp_idx + it_n;
        if (isInMap(neighbor_idx)) {
          int neighbor_adr = toAddress(neighbor_idx);
          md_->occupancy_buffer_inflate_[neighbor_adr] = getOriOccupancy(tmp_idx);
        }
      }
    }
    auto t2 = ros::Time::now();
    ROS_WARN_THROTTLE(1, "mapper with inflation t: %lf", (t2 - t1).toSec());
  }
}


void SDFMap::decayLocalCloud(const Eigen::Vector3d& pos,
                                  double max_decay_range) {

  Eigen::Vector3d min_pos, max_pos;
  Eigen::Vector3i min_id, max_id;
  min_pos(0) = pos(0) - max_decay_range;
  min_pos(1) = pos(1) - max_decay_range;
  min_pos(2) = pos(2) - max_decay_range;
  max_pos(0) = pos(0) + max_decay_range;
  max_pos(1) = pos(1) + max_decay_range;
  max_pos(2) = pos(2) + max_decay_range;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  auto t_now = ros::Time::now();
  double dt = (t_now - last_decay_time_).toSec();
  // Decaying voxels within robot's local region (voxels will disappear if
  // unobserved for (val_occ - val_even) / val_decay times)
  for (int x = min_id(0); x <= max_id(0); ++x) {
    for (int y = min_id(1); y <= max_id(1); ++y) {
      for (int z = min_id(2); z <= max_id(2); ++z) {
        // Do not decay if in no flight zone
        if (mp_->set_no_flight_zone_) {
          if (isInNoFlightZone(Eigen::Vector3i(x, y, z))) continue;
        }

        if (md_->occupancy_buffer_[toAddress(x, y, z)] > mp_->min_occupancy_log_ - mp_->decay_amount_lower_than_occ_log_) {
          md_->occupancy_buffer_[toAddress(x, y, z)] -= val_decay_per_sec_*dt;
        }
        if (md_->occupancy_buffer_inflate_[toAddress(x, y, z)] > mp_->min_occupancy_log_ - mp_->decay_amount_lower_than_occ_log_) {
          md_->occupancy_buffer_inflate_[toAddress(x, y, z)] -= val_decay_per_sec_*dt;
        }
      }
    }
  }
}

Eigen::Vector3d
SDFMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_->map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_->map_min_boundary_ - camera_pt;
  double min_t = 1000000;
  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;
      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }
  return camera_pt + (min_t - 1e-3) * diff;
}


double SDFMap::getResolution() {
  return mp_->resolution_;
}

int SDFMap::getVoxelNum() {
  return mp_->map_voxel_num_[0] * mp_->map_voxel_num_[1] * mp_->map_voxel_num_[2];
}

void SDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
  ori = mp_->map_origin_, size = mp_->map_size_;
}

void SDFMap::getBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax) {
  bmin = mp_->box_mind_;
  bmax = mp_->box_maxd_;
}

void SDFMap::getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax, bool reset) {
  bmin = md_->update_min_;
  bmax = md_->update_max_;
  if (reset) md_->reset_updated_box_ = true;
}

int SDFMap::getMapDefault() {
  // return mp_->clamp_min_log_ - mp_->unknown_flag_;
  return UNKNOWN;
}

}  // namespace ms_planner
// SDFMap
