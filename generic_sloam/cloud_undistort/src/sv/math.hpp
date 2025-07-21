#pragma once

#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <numeric>
#include <type_traits>

#include "sv_core/logging.hpp"
#include "sv_core/types.hpp"

namespace sv {

/// Useful constants
namespace bmc = boost::math::constants;
constexpr auto kPi = bmc::pi<real_t>();
constexpr auto kTau = bmc::two_pi<real_t>();
constexpr auto kPi_2 = bmc::half_pi<real_t>();

/// Degree from Radian
inline constexpr real_t Deg_Rad(real_t rad) noexcept { return rad * 180 / kPi; }

/// Radian from Degree
inline constexpr real_t Rad_Deg(real_t deg) noexcept { return deg / 180 * kPi; }

/// A (real) closed interval, boost interval is too heavy
template <typename T = real_t>
struct Interval {
  Interval() = default;
  Interval(const T &left, const T &right) : left_(left), right_(right) {
    CHECK_LE(left, right);
  }

  T left_, right_;

  const T &a() const noexcept { return left_; }
  const T &b() const noexcept { return right_; }
  T width() const noexcept { return b() - a(); }
  bool empty() const noexcept { return b() <= a(); }
  bool contains(T v) const noexcept { return (a() <= v) && (v <= b()); }

  /// Whether this interval contains other
  bool contains(const Interval<T> &other) const noexcept {
    return a() <= other.a() && other.b() <= b();
  }

  /// Normalize v to [0, 1], assumes v in [left, right]
  /// Only enable if we have floating type
  T Normalize(const T &v) const {
    static_assert(std::is_floating_point<T>::value, "Must be floating point");
    CHECK(contains(v));
    return (v - a()) / width();
  }

  /// InvNormalize v to [left, right], assumes v in [0, 1]
  T InvNormalize(const T &v) const {
    static_assert(std::is_floating_point<T>::value, "Must be floating point");
    CHECK_LE(0, v);
    CHECK_LE(v, 1);
    return v * width() + a();
  }
};

/**
 * @brief CartesianProduct
 * @param vs each v in vs is a set
 * @return v1 x v2 x ... x vn
 */
template <typename T>
std::vector<std::vector<T>> CartesianProductHelper(
    const std::vector<std::vector<T>> &vs) {
  std::vector<std::vector<T>> s = {{}};
  for (auto &u : vs) {
    std::vector<std::vector<T>> r;
    for (auto &x : s) {
      for (auto y : u) {
        r.push_back(x);
        r.back().push_back(y);
      }
    }
    s.swap(r);
  }

  // Just to make sure
  const auto size = std::accumulate(
      vs.cbegin(), vs.cend(), 1,
      [](const auto &a, const auto &b) { return a * b.size(); });
  CHECK_EQ(s.size(), size);

  return s;
}

/// Convert col major matrix to vector of vector
template <typename D>
std::vector<std::vector<typename D::Scalar>> VecOfVec_ColMatrix(
    const Eigen::DenseBase<D> &m) {
  std::vector<std::vector<typename D::Scalar>> v(m.cols());
  for (int j = 0; j < m.cols(); ++j) {
    for (int i = 0; i < m.rows(); ++i) {
      v[j].push_back(m(i, j));
    }
  }
  return v;
}

/// Convert vector of vector to col major matrix
template <typename T>
MatrixXXT<T> ColMatrix_VecOfVec(const std::vector<std::vector<T>> &v) {
  const auto n_cols = v.size();
  const auto n_rows = v.empty() ? 0 : v[0].size();

  MatrixXXT<T> m(n_rows, n_cols);

  for (size_t j = 0; j < n_cols; ++j) {
    CHECK_EQ(v[j].size(), n_rows);
    for (size_t i = 0; i < n_rows; ++i) {
      m(i, j) = v[j][i];
    }
  }
  return m;
}

/// Eigen version, assume each v has the same size
template <typename D>
MatrixXXT<typename D::Scalar> CartesianProduct(const Eigen::DenseBase<D> &vs) {
  const auto vv = VecOfVec_ColMatrix(vs);
  const auto cp = CartesianProductHelper(vv);
  return ColMatrix_VecOfVec(cp);
}

/// Dehomogenized (x, y, z) -> (x/z, y/z, 1)
/// Vectorized version
static MatrixXX DehomogenizedColwise(const MatrixXX &m) {
  CHECK_GT(m.rows(), 2);
  CHECK((m.array().bottomRows<1>() > 0).all());
  return m.array().rowwise() / m.array().bottomRows<1>();
}

template <typename T, int N>
VectorNT<T, N> Dehomogenized(const VectorNT<T, N> &r) {
  static_assert(N > 2, "N > 2 in Dehomogenize");
  return r / r(N - 1);
}

inline Vector3 Dehomogenized(const EigenCRef<Vector3> &r3) {
  return r3 / r3(2);
}

/// Project vector from u to v
static Vector3 ProjectVectorOnVector(const Vector3 &u, const Vector3 &v) {
  const auto v_norm = v.norm();
  return u.dot(v) / (v_norm * v_norm) * v;
}

/// Project to plane defined by normal
static Vector3 ProjectVectorOnPlane(const Vector3 &u, const Vector3 &n) {
  return u - ProjectVectorOnVector(u, n);
}

}  // namespace sv
