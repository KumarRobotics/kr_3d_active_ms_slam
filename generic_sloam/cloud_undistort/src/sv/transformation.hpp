#pragma once

#include <sophus/interpolate.hpp>

#include "utils.hpp"
// #include <sv_core/macros.hpp>

namespace sv {

/// Make a grid in tangent space
Matrix6X MakeSE3TangentGrid(double t_step, double r_step);

/// Apply tangent grid to SE3
// TODO: can be generic
SE3Vector MakeSE3Grid(const SE3& T, const Matrix6X& tangents);

/// Flatten a vector of sophus type to matrix each column is element in vector
template <typename SophusT>
MatrixXX EigenMatrix_SophusVec(const AlignedVector<SophusT>& vec) {
  const auto n = vec.size();
  constexpr auto m = SophusT::num_parameters;
  MatrixXX mat(m, n);
  for (size_t j = 0; j < n; ++j) {
    const auto* ptr = vec[j].data();
    for (size_t i = 0; i < m; ++i) {
      mat(i, j) = ptr[i];
    }
  }
  return mat;
}

/// Map a vector of sophus type to matrix each column is one element in vector
template <typename SophusT>
EigenCMap<MatrixXX> EigenMap_SophusVec(const AlignedVector<SophusT>& vec) {
  SV_STATIC_ASSERT_IS_REAL(typename SophusT::Scalar);
  return EigenCMap<MatrixXX>(vec[0].data(), SophusT::num_parameters,
                             vec.size());
}

/// Generate a rotation such that we rotate axis at src to look at dst
/// Equivalent to R_01
inline SO3 SO3LookAt(const SO3::Point& src, const SO3::Point& dst,
                     const SO3::Point& axis = SO3::Point(0, 0, 1)) {
  return SO3{Quat::FromTwoVectors(axis, dst - src)};
}

template <typename SophusT>
AlignedVector<SophusT> GroupInverted(const AlignedVector<SophusT>& Ts) {
  AlignedVector<SophusT> Ts_inv;
  Ts_inv.reserve(Ts.size());
  for (const auto& T : Ts) Ts_inv.push_back(T.inverse());
  return Ts_inv;
}

template <typename SophusT>
void GroupInvert(AlignedVector<SophusT>& Ts) {
  for (auto& T : Ts) T = T.inverse();
}

// Vectorized group action
inline Matrix3X GroupAction(const SO3& R, const Matrix3X& points) {
  return R.matrix() * points;
}

inline Matrix3X GroupAction(const SE3& T, const Matrix3X& points) {
  auto points_trans = GroupAction(T.so3(), points);
  points_trans.colwise() += T.translation();
  return points_trans;
}

}  // namespace sv
