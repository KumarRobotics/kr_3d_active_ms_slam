#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <boost/optional.hpp>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <vector>

#include "eigen_checks.hpp"

// Aligned std::vector
template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

using SE3 = Sophus::SE3d;
using SO3 = Sophus::SO3d;
using SE2 = Sophus::SE2d;
using SO2 = Sophus::SO2d;
using Quat = Eigen::Quaterniond;
using Vector3 = Eigen::Matrix<double, 3, 1>;

template <typename T>
struct Timed {
  Timed() = default;
  Timed(double t, const T &d) : time(t), data(d) {}
  double time{0.0};
  T data;
};

using TimedSO3 = Timed<SO3>;

// optional
template <typename T>
using Optional = boost::optional<T>;
using boost::make_optional;

/// A (real) closed interval, boost interval is too heavy
template <typename T = double>
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

#define SV_CHECK_PCL_POINT_XYZ(POINT)                        \
  BOOST_MPL_ASSERT_MSG((pcl::traits::has_xyz<POINT>::value), \
                       POINT_TYPE_SHOULD_HAVE_XYZ_FIELD, (POINT))

/// Check if a point is nan
template <typename PointT>
constexpr bool PclPointIsNaN(const PointT &point) {
  SV_CHECK_PCL_POINT_XYZ(PointT);
  return std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z);
}

template <typename T>
T clip(const T &n, const T &lower, const T &upper) {
  return std::max(lower, std::min(n, upper));
}