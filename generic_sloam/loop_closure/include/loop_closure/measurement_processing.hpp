#ifndef MEASUREMENT_NOBEARING_H_
#define MEASUREMENT_NOBEARING_H_

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

bool in_region(const double& roll_vio, const double& pitch_vio,
               const std::vector<Eigen::Vector3d>& relative_msts,
               const std::vector<Eigen::Vector3d>& ugv_positions,
               std::vector<int>& matched_indices_out,
               Eigen::Vector3d& position_estimate_out, double& yaw_estimate_out,
               Eigen::Matrix3d& best_rotation_matrix,
               const double& max_tol_per_landmark_pos_error = 0.3);

bool estimate_bearingpos(const std::vector< Eigen::Vector3d>& measurements, const std::vector< Eigen::Vector3d>& obj_positions, bool fit_all_available,
                            std::vector< std::array< int,2>>& matched_indices_out, Eigen::Vector3d& position_estimate_out, double& yaw_estimate_out, int& exit_sign, const double& max_tol_per_landmark_pos_error=0.3);

bool estimate_bearingpos(const std::vector< Eigen::Vector3d>& measurements, const std::vector< Eigen::Vector3d>& obj_positions, bool fit_all_available, double yaw_center, double yaw_max_dev, 
                            std::vector< std::array< int,2>>& matched_indices_out, Eigen::Vector3d& position_estimate_out, double& yaw_estimate_out, int& exit_sign, const double& max_tol_per_landmark_pos_error=0.3);


#endif