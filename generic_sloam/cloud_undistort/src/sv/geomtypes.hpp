#pragma once

#include <kindr/rotations/Rotation.hpp>
#include <kindr/rotations/RotationDiff.hpp>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sv_core/types.hpp>

namespace sv {

// sophus real type
using SE3 = Sophus::SE3d;
using SO3 = Sophus::SO3d;
using SE2 = Sophus::SE2d;
using SO2 = Sophus::SO2d;
using Quat = Eigen::Quaterniond;

using SE3Vector = AlignedVector<SE3>;

template <typename T>
using TimedT = std::pair<real_t, T>;

template <typename T>
struct Timed {
  Timed() = default;
  Timed(real_t t, const T& d) : time(t), data(d) {}
  real_t time{0.0};
  T data;
};

using TimedSO3 = Timed<SO3>;

// kindr
/// VERY IMPORTANT NOTE:
/// In our work, we use euler angle rpy (extrinsic), which is euler angle zyx
/// (intrinsic), which is the same as kindr::EulerAngleZYX. Thus, if we use erpy
/// to represent such euler angle, then it will be in the order of (r, p, y) and
/// when constructing a kindr::EulerAngleZYX type, we have to make sure we flip
/// the vector because, it takes (y, p, r) as input. The only euler angle
/// representation we will use from Kindr is the EulerAngleZYX one.
namespace kd = kindr;
using EulerAnglesZyx = kindr::EulerAnglesZyxD;
using EulerAnglesZyxDiff = kindr::EulerAnglesZyxDiffD;
using LocalAngularVelocity = kindr::LocalAngularVelocityD;
using RotationMatrix = kindr::RotationMatrixD;

}  // namespace sv
