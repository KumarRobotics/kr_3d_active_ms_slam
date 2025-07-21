#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>

#include <boost/circular_buffer.hpp>
#include <cmath>
#include <sophus/interpolate.hpp>

#include "conversion.hpp"
#include "definitions.hpp"

namespace sv::cloud {

using namespace sensor_msgs;
using TimeInterval = Interval<double>;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

/**
 * @brief ImusToTimedSO3s
 * @param imu_msgs
 * @param interval
 * @param R_t_i rotation from imu to target frame
 * @return
 */
AlignedVector<TimedSO3> ImusToTimedSO3s(const std::vector<Imu> &imus,
                                        const TimeInterval &interval,
                                        const SO3 &R_t_i = SO3());
/**
 * @brief UndistortCloud
 * @param cloud
 * @param Rs_0_t
 * @param delta_t
 */
void UndistortCloud(Cloud &cloud, const AlignedVector<TimedSO3> &Rs_0_t,
                    double delta_t);

/// CloudUndistortNode
class CloudUndistortNode {
 public:
  explicit CloudUndistortNode(const ros::NodeHandle &pnh);

  void ImuCb(const ImuConstPtr &imu_msg);
  void CloudCb(const PointCloud2ConstPtr &cloud_msg);

 private:
  /// look up transform R_l_i
  bool LookupTransform(const std::string &cloud_frame);
  /// Get imu messages from buffer
  std::vector<Imu> GetImus(const TimeInterval &time_interval) const;

  /// ROS
  ros::NodeHandle pnh_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_cloud_;
  ros::Publisher pub_cloud_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string imu_frame_;

  /// Other
  boost::circular_buffer<Imu> imu_buffer_;
  Optional<SO3> R_l_i_;
  double firing_cycle_;
  double prev_time_;
};

CloudUndistortNode::CloudUndistortNode(const ros::NodeHandle &pnh)
    : pnh_(pnh), tf_listener_(tf_buffer_), imu_buffer_(500), prev_time_(0) {
  sub_cloud_ = pnh_.subscribe("cloud", 1, &CloudUndistortNode::CloudCb, this);
  sub_imu_ = pnh_.subscribe("imu", 100, &CloudUndistortNode::ImuCb, this);
  pub_cloud_ = pnh_.advertise<Cloud>("cloud_undistort", 5);

  // firing_cycle_ = 55.296 * 1e-6;
  // Ouster: 1 scan with 2048 activations every 0.1s; 0.1/2048 = firing cycle
  firing_cycle_ = 4.8828 * 1e-5;

  auto logLevel = ros::console::levels::Info;
  if (pnh_.param("debug", false)) {
    logLevel = ros::console::levels::Debug;
  }

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, logLevel)) {
    ros::console::notifyLoggerLevelsChanged();
  }
}

void CloudUndistortNode::ImuCb(const ImuConstPtr &imu_msg_ptr) {
  const Imu &imu_msg = *imu_msg_ptr;
  if (imu_frame_.empty()) {
    imu_frame_ = imu_msg.header.frame_id;
    ROS_INFO("Imu frame is set to %s", imu_frame_.c_str());
  } else {
    if (imu_msg.header.stamp < imu_buffer_.back().header.stamp) {
      ROS_WARN("Current imu %f comes before the last imu %f, clear buffer",
               imu_msg.header.stamp.toSec(),
               imu_buffer_.back().header.stamp.toSec());
      imu_buffer_.clear();
    }
  }

  imu_buffer_.push_back(imu_msg);
}

void CloudUndistortNode::CloudCb(const PointCloud2ConstPtr &cloud_msg) {
  if (imu_frame_.empty()) return;

  // Look up transform first if we don't already have it
  // Skip if lookup failed
  if (!R_l_i_) {
    if (!LookupTransform(cloud_msg->header.frame_id)) return;
  }

  // Now we must have a transform
  // CHECK(R_l_i_) << "no R_l_i";

  Cloud::Ptr cloud(new Cloud);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  if (!cloud->isOrganized()) {
    ROS_ERROR("Cloud is not organized, not distorting");
    return;
  }
  ROS_DEBUG("cloud %zu = %u x %u", cloud->size(), cloud->height, cloud->width);

  // Given the current time and firing cycle, compute time delta between first
  // and last firing
  const double time_begin = cloud_msg->header.stamp.toSec();
  double cycle_time = firing_cycle_ * (cloud->width - 1);
  const double time_end = time_begin + cycle_time;
  ROS_DEBUG("Cloud time interval mine: [%f, %f], width: %f", time_begin,
            time_end, time_end - time_begin);
  const TimeInterval cloud_time_interval(time_begin, time_end);
  ROS_DEBUG("Cloud time interval: [%f, %f], width %f", cloud_time_interval.a(),
            cloud_time_interval.b(), cloud_time_interval.width());

  // Given this time interval, we get imu msgs with in the interval from buffer
  const auto imus = GetImus(cloud_time_interval);
  ROS_DEBUG("Got %zu imu msgs", imus.size());
  if (imus.empty()) {
    ROS_WARN("Failed to get imu msgs, not distorting");
    return;
  }

  // Check imu time against cloud time interval
  ROS_DEBUG("imu cloud delta time %f, %f",
            imus.front().header.stamp.toSec() - cloud_time_interval.a(),
            cloud_time_interval.b() - imus.back().header.stamp.toSec());

  const auto Rs_0_t = ImusToTimedSO3s(imus, cloud_time_interval, *R_l_i_);
  ROS_DEBUG("Starting undistort");
  UndistortCloud(*cloud, Rs_0_t, firing_cycle_);
  pub_cloud_.publish(cloud);

  // Record previous time just for debug
  if (prev_time_ > 0) {
    ROS_DEBUG("cloud time diff: %f", time_begin - prev_time_);
  }
  prev_time_ = time_begin;
}

bool CloudUndistortNode::LookupTransform(const std::string &cloud_frame) {
  geometry_msgs::TransformStamped tf_msg;
  R_l_i_ = SO3();
  return true;
  // try {
  //   tf_msg = tf_buffer_.lookupTransform(cloud_frame, imu_frame_,
  //   ros::Time(0)); R_l_i_ = ToSO3(tf_msg.transform.rotation);
  //   ROS_INFO_STREAM("Got R_l_i: \n" << R_l_i_->matrix());
  //   return true;
  // } catch (tf2::TransformException &ex) {
  //   ROS_WARN("%s", ex.what());
  //   return false;
  // }
}

std::vector<Imu> CloudUndistortNode::GetImus(
    const TimeInterval &interval) const {
  const auto &buffer = imu_buffer_;
  std::vector<Imu> imu_msgs;

  auto is_inside = [&](const sensor_msgs::Imu &imu_msg) {
    return interval.contains(imu_msg.header.stamp.toSec());
  };

  // Get all the imu message that falls in this time interval
  std::copy_if(std::cbegin(buffer), std::cend(buffer),
               std::back_inserter(imu_msgs), is_inside);

  //  boost::push_back(imu_msgs, buffer | boost::adaptors::filtered(is_inside));

  return imu_msgs;
}

AlignedVector<TimedSO3> ImusToTimedSO3s(const std::vector<Imu> &imus,
                                        const TimeInterval &interval,
                                        const SO3 &R_t_i) {
  AlignedVector<TimedSO3> Rs_0_t;
  if (imus.empty()) return Rs_0_t;

  Rs_0_t.reserve(imus.size() + 2);
  // First rotation is always identity
  Rs_0_t.emplace_back(interval.a(), SO3{});

  // For undistortion assume cloud start is identity
  // |----------|----------|----------|
  // t_c0  |    t_c1       ...        t_ck
  // |     |    |    |     |    |     |
  // |     i0   i1   i2    ...  ik    |
  // R0    R1   R2   R3    ...  Rk+1  Rk+2
  // R0 --- Rk+2 are what we need

  // Use simple integration
  // Get a copy of the first imu, with time 0
  Imu imu_km1 = imus.front();  // i_{k-1}
  imu_km1.header.stamp = ros::Time(interval.a());
  Vector3 w_km1;
  tf2::fromMsg(imu_km1.angular_velocity, w_km1);

  // Compute incrementatal rotations between each imu
  for (size_t k = 0; k < imus.size(); ++k) {
    const auto &imu_k = imus[k];  // i_k

    Vector3 w_k;
    tf2::fromMsg(imu_k.angular_velocity, w_k);

    const Vector3 w = R_t_i * (w_km1 + w_k) / 2.0;  // imu frame -> cloud frame
    const auto dt = (imu_k.header.stamp - imu_km1.header.stamp).toSec();
    // R_0_tk = R_0_tkm1 * R_tkm1_tk
    const SO3 R_0_tk = Rs_0_t.back().data * SO3::exp(w * dt);
    Rs_0_t.emplace_back(imu_k.header.stamp.toSec(), R_0_tk);
    imu_km1 = imu_k;
    w_km1 = w_k;
  }

  // Last one
  const double dt = interval.b() - imu_km1.header.stamp.toSec();
  const Vector3 w = R_t_i * w_km1;
  const SO3 R_0_tk = Rs_0_t.back().data * SO3::exp(w * dt);

  Rs_0_t.emplace_back(interval.b(), R_0_tk);

  return Rs_0_t;
}

void UndistortCloud(Cloud &cloud, const AlignedVector<TimedSO3> &Rs_0_t,
                    double delta_t) {
  if (Rs_0_t.empty()) return;
  if (cloud.empty()) return;

  // Start time
  const auto t0 = Rs_0_t.front().time;
  // This index into Rs
  int k = 1;

  // Iterate through cloud column
  for (int j = 0; j < cloud.width; ++j) {
    // Get time of this point
    const double t = t0 + j * delta_t;

    // Use col_time to retrieve two poses
    // while col time is bigger than current time, increment k
    while (t > Rs_0_t[k].time && k < Rs_0_t.size() - 1) ++k;

    const auto &R_0_tkm1 = Rs_0_t[k - 1];
    const auto &R_0_tk = Rs_0_t[k];
    CHECK_GE(t, R_0_tkm1.time);
    CHECK_LE(t, R_0_tk.time);

    // Normalize t and use it to interpolate
    const double t_norm =
        clip((t - R_0_tkm1.time) / (R_0_tk.time - R_0_tkm1.time), 0.0, 1.0);

    const SO3 R_0_t = Sophus::interpolate(R_0_tkm1.data, R_0_tk.data, t_norm);

    for (uint32_t i = 0; i < cloud.height; ++i) {
      auto &p = cloud(j, i);
      // Skip nan point
      if (PclPointIsNaN(p)) continue;
      // transform point to frame 0
      p.getVector3fMap() = R_0_t.cast<float>() * p.getVector3fMap();
    }
  }
  ROS_DEBUG("out undistort");
}

}  // namespace sv::cloud

int main(int argc, char **argv) {
  ros::init(argc, argv, "range_undistort_node");

  sv::cloud::CloudUndistortNode node(ros::NodeHandle("~"));
  ros::spin();
}
