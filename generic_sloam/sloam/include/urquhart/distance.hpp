#pragma once

#include <definitions.h>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <algorithm>
#include <iostream>
#include <numeric>
#include <tuple>
#include <utility>
#include <vector>

using vecPtT = std::vector<double>;
using EdgeT = std::pair<int, int>;
using PointVector = std::vector<vecPtT>;
using ptPair = std::pair<vecPtT, vecPtT>;
using idxPair = std::pair<size_t, size_t>;

// using PointPCL  = pcl::PointXYZI;
// using CloudT = pcl::PointCloud<pcl::PointXYZI>;
inline float pow_2(const float& x) { return x * x; }

inline double euclideanDistance(std::vector<double> A, std::vector<double> B) {
  double d = 0;
  for (size_t i = 0; i < A.size(); ++i) {
    d += pow_2(A[i] - B[i]);
  }
  return std::sqrt(d);
}

inline double euclideanDistance2D(std::vector<double> A,
                                  std::vector<double> B) {
  double d = 0;
  for (size_t i = 0; i < 2; ++i) {
    d += pow_2(A[i] - B[i]);
  }
  return std::sqrt(d);
}

inline float euclideanDistance(const PointT& A, const PointT& B) {
  return std::sqrt(pow_2(A.x - B.x) + pow_2(A.y - B.y) + pow_2(A.z - B.z));
}

inline float euclideanDistance2D(const PointT& A, const PointT& B) {
  return std::sqrt(pow_2(A.x - B.x) + pow_2(A.y - B.y));
}

inline size_t cantorPairing(size_t a, size_t b) {
  // a is always the smallest number
  size_t aux = a;
  if (b < a) {
    a = b;
    b = aux;
  }
  return (a + b) * (a + b + 1) / 2 + a;
}

inline bool isOrthogonal(const Eigen::Matrix4f& tf) {
  Eigen::Matrix3f R = tf.block<3, 3>(0, 0);
  float res = (R * R.transpose() - Eigen::Matrix3f::Identity()).norm();
  return res < 1e-10;
}