#pragma once

#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <functional>

namespace sv::cloud {

#define SV_CHECK_PCL_POINT_XYZ(POINT)                        \
  BOOST_MPL_ASSERT_MSG((pcl::traits::has_xyz<POINT>::value), \
                       POINT_TYPE_SHOULD_HAVE_XYZ_FIELD, (POINT))

static constexpr auto kPclNaN = std::numeric_limits<float>::quiet_NaN();

static constexpr float PointAltitude(float x, float y, float z) {
  // [-pi, pi]
  return std::atan2(z, std::hypot(x, y));
}

/**
 * @brief Compute altitude of a point in radius, assumes point in local frame
 * @details Altitude is defined as the angle between the xy-plane and the point
 * https://en.wikipedia.org/wiki/Horizontal_coordinate_system
 * @param point
 */
template <typename PointT>
float PclPointAltitude(const PointT &point) {
  SV_CHECK_PCL_POINT_XYZ(PointT);
  return PointAltitude(point.x, point.y, point.z);
}

static constexpr float PointAzimuth(float x, float y, float) {
  const auto a = std::atan2(y, x);
  // Convert to [0, 2pi)
  return y >= 0 ? a : a + M_PI * 2;
}

/**
 * @brief Compute azimuth of a point in radius, assumes point in local frame
 * @details Azimuth is defined as the angle between the projection of the point
 * and the north, https://en.wikipedia.org/wiki/Horizontal_coordinate_system
 * @param point
 */
template <typename PointT>
float PclPointAzimuth(const PointT &point) {
  SV_CHECK_PCL_POINT_XYZ(PointT);
  return PointAzimuth(point.x, point.y, point.z);
}

static constexpr float PointRange(float x, float y, float z) {
  return std::sqrt(x * x + y * y + z * z);
}

/// Compute range of point by L2 norm
template <typename PointT>
double PclPointRange(const PointT &point) {
  SV_CHECK_PCL_POINT_XYZ(PointT);
  return PointRange(point.x, point.y, point.z);
}

/// Check if a point is finite
template <typename PointT>
constexpr bool PclPointIsFinite(const PointT &point) {
  SV_CHECK_PCL_POINT_XYZ(PointT);
  return pcl_isfinite(point.x) && pcl_isfinite(point.y) &&
         pcl_isfinite(point.z);
}

/// Check if a point is nan
template <typename PointT>
constexpr bool PclPointIsNaN(const PointT &point) {
  SV_CHECK_PCL_POINT_XYZ(PointT);
  return pcl_isnan(point.x) || pcl_isnan(point.y) || pcl_isnan(point.z);
}

/// Set a point to NaN
template <typename PointT>
void PclSetPointNaN(PointT &point) {
  SV_CHECK_PCL_POINT_XYZ(PointT);
  point.x = point.y = point.z = kPclNaN;
}

/// Remove Nan points
template <typename PointT>
std::vector<int> PclRemoveNan(pcl::PointCloud<PointT> &cloud) {
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(cloud, cloud, indices);
  return indices;
}

/// Copy cloud meta information
template <typename PointT>
void PclCopyCloudMeta(const pcl::PointCloud<PointT> &cloud_in,
                      pcl::PointCloud<PointT> &cloud_out) {
  if (&cloud_in != &cloud_out) {
    cloud_out.header = cloud_in.header;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.sensor_origin_ = cloud_in.sensor_origin_;
    cloud_out.sensor_orientation_ = cloud_out.sensor_orientation_;
  }
}

/// Remove point based on a unary predicated
template <typename PointT, typename UnaryPred>
void PclRemovePointIf(const pcl::PointCloud<PointT> &cloud_in,
                      pcl::PointCloud<PointT> &cloud_out,
                      const UnaryPred &pred) {
  if (&cloud_in == &cloud_out) {
    // in and out are just the same, filter inplace
    cloud_out.erase(std::remove_if(cloud_out.begin(), cloud_out.end(), pred),
                    cloud_out.end());
  } else {
    // in and out are not the same, filter outside
    typename pcl::PointCloud<PointT>::VectorType points;
    points.reserve(cloud_in.size());
    std::copy_if(cloud_in.points.begin(), cloud_in.points.end(),
                 std::back_inserter(points),
                 [&](const auto &p) { return !pred(p); });
    // just swap the points
    cloud_out.points.swap(points);
    cloud_out.width = cloud_out.points.size();
    cloud_out.height = 1;

    // Don't forget to copy the metadata
    PclCopyCloudMeta(cloud_in, cloud_out);
  }
}

/// Polar 3d coordinate system
/// altitude up, azimuth counter-clockwise
struct Polar3D {
  Polar3D() = default;

  explicit Polar3D(float altitude, float azimuth, float range)
      : altitude(altitude), azimuth(azimuth), range(range) {}

  template <typename Point>
  explicit Polar3D(const Point &point)
      : altitude(PclPointAltitude(point)),
        azimuth(PclPointAzimuth(point)),
        range(PclPointRange(point)) {}

  pcl::PointXYZ ToPoint() const noexcept {
    using std::cos;
    using std::sin;

    const auto ca = cos(altitude);
    return pcl::PointXYZ(ca * cos(azimuth) * range, ca * sin(azimuth) * range,
                         sin(altitude) * range);
  }

  float altitude, azimuth, range;
};

}  // namespace sv::cloud
