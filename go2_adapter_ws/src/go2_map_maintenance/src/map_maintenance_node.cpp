#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include "go2_map_maintenance/msg/map_snapshot.hpp"
#include "go2_map_maintenance/cloud_decode.hpp"

namespace go2_map_maintenance {
using Cloud = sensor_msgs::msg::PointCloud2;
using Odom = nav_msgs::msg::Odometry;
using Snapshot = msg::MapSnapshot;
using Steady = std::chrono::steady_clock;

class MapMaintenance : public rclcpp::Node {
 public:
  MapMaintenance() : Node("go2_map_maintenance") {
    frame_ = declare_parameter("planning_frame", std::string("odom"));
    base_ = declare_parameter("base_frame", std::string("base_link"));
    const auto cloud_topic = declare_parameter("cloud_topic", std::string("/pct_go2/cloud"));
    const auto odom_topic = declare_parameter("odom_topic", std::string("/pct_go2/body_pose"));
    Limits limits;
    limits.voxel_size = positive("voxel_size", 0.10);
    limits.extent_xy = positive("extent_xy_m", 10.0);
    limits.extent_z = positive("extent_z_m", 4.0);
    limits.candidate_ttl = positive("candidate_ttl_sec", 5.0);
    limits.min_observations = integer("min_observations", 3);
    limits.max_voxels = integer("max_voxels", 200000);
    limits.max_input_points = integer("max_input_points", 200000);
    max_input_points_ = limits.max_input_points;
    min_points_ = integer("min_confirmed_points", 500);
    if (min_points_ > limits.max_voxels || frame_.empty() || base_.empty())
      throw std::invalid_argument("invalid map frames or confirmed-point limit");
    map_ = std::make_unique<VoxelMap>(limits);
    source_timeout_ = positive("source_timeout_sec", 0.5);
    skew_ = positive("max_pose_cloud_skew_sec", 0.10);
    translation_jump_ = positive("translation_jump_m", 0.5);
    rotation_jump_ = positive("rotation_jump_rad", 0.7853981633974483);
    const double integrate_hz = positive("integration_rate_hz", 5.0);
    const double publish_hz = positive("publish_rate_hz", 1.0);
    if (publish_hz > 20 || integrate_hz > 100 || source_timeout_ <= 1.0 / integrate_hz)
      throw std::invalid_argument("map rate is outside supported limits");
    snapshot_pub_ = create_publisher<Snapshot>("/pct_go2/map_snapshot", 1);
    cloud_pub_ = create_publisher<Cloud>("/pct_go2/accumulated_cloud", 1);
    status_pub_ = create_publisher<std_msgs::msg::String>("/pct_go2/map_status", 1);
    odom_sub_ = create_subscription<Odom>(odom_topic, rclcpp::SensorDataQoS().keep_last(50),
      [this](Odom::ConstSharedPtr value) { on_odom(*value); });
    cloud_sub_ = create_subscription<Cloud>(cloud_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](Cloud::ConstSharedPtr value) {
        if (value->header.frame_id != frame_ || !fresh(value->header.stamp)) {
          cloud_error_ = "cloud_frame_or_timestamp_invalid";
          pending_.reset();
          return;
        }
        pending_ = value;  // Single pending frame; no unbounded queue.
      });
    reset_sub_ = create_subscription<std_msgs::msg::Empty>("/pct_go2/reset_map", 1,
      [this](std_msgs::msg::Empty::ConstSharedPtr) { reset(); publish(); });
    integrate_timer_ = create_wall_timer(period(integrate_hz), [this]() { integrate(); });
    publish_timer_ = create_wall_timer(period(publish_hz), [this]() { publish(); });
    reset();
    RCLCPP_INFO(get_logger(), "Static-scene accumulation in %s; no SLAM or free-space carving",
                frame_.c_str());
  }

 private:
  struct Pose {
    int64_t stamp;
    Point position;
    std::array<double, 4> q;
  };
  double positive(const std::string& name, double value) {
    const double result = declare_parameter(name, value);
    if (!std::isfinite(result) || result <= 0)
      throw std::invalid_argument(name + " must be finite and positive");
    return result;
  }
  size_t integer(const std::string& name, int64_t value) {
    const int64_t result = declare_parameter<int64_t>(name, value);
    if (result <= 0 || result > 10000000)
      throw std::invalid_argument(name + " must be in [1, 10000000]");
    return static_cast<size_t>(result);
  }
  static std::chrono::nanoseconds period(double hz) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1 / hz));
  }
  static double monotonic() {
    return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();
  }
  static int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
    if (stamp.nanosec >= 1000000000U) return 0;
    return int64_t(stamp.sec) * 1000000000LL + stamp.nanosec;
  }
  bool fresh(const builtin_interfaces::msg::Time& stamp) {
    const int64_t ns = stamp_ns(stamp);
    const double age = (get_clock()->now().nanoseconds() - ns) * 1e-9;
    return ns > 0 && age >= -0.05 && age <= source_timeout_;
  }
  void reset() {
    map_->reset();
    poses_.clear(); pending_.reset();
    fault_.clear(); cloud_error_.clear(); odom_error_.clear();
    last_input_ = last_odom_ = -std::numeric_limits<double>::infinity();
    cache_version_ = 0; cached_ = Cloud();
    const auto tick = std::chrono::duration_cast<std::chrono::nanoseconds>(
      Steady::now().time_since_epoch()).count();
    const auto system = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    epoch_ = std::to_string(system) + "-" + std::to_string(tick) + "-" + std::to_string(++resets_);
  }
  void on_odom(const Odom& value) {
    if (!fault_.empty()) return;
    if (value.header.frame_id != frame_ || value.child_frame_id != base_ ||
        !fresh(value.header.stamp)) {
      odom_error_ = "odom_frame_or_timestamp_invalid"; return;
    }
    const auto& p = value.pose.pose.position;
    const auto& q = value.pose.pose.orientation;
    Pose pose{stamp_ns(value.header.stamp), {p.x, p.y, p.z}, {q.x, q.y, q.z, q.w}};
    double norm = 0;
    for (double number : pose.q) norm += number * number;
    if (!finite(pose.position) || !std::isfinite(norm) || norm < 1e-12) {
      odom_error_ = "nonfinite_or_invalid_odometry"; return;
    }
    norm = std::sqrt(norm);
    for (double& number : pose.q) number /= norm;
    if (!poses_.empty()) {
      const Pose& previous = poses_.back();
      if (pose.stamp <= previous.stamp) return;
      const double distance = std::hypot(std::hypot(p.x-previous.position.x,
          p.y-previous.position.y), p.z-previous.position.z);
      double dot = 0;
      for (size_t i = 0; i < 4; ++i) dot += pose.q[i] * previous.q[i];
      const double angle = 2 * std::acos(std::min(1.0, std::abs(dot)));
      if (distance > translation_jump_ || angle > rotation_jump_) {
        fault_ = "odometry_jump_reset_required";
        pending_.reset(); publish(); return;
      }
    }
    poses_.push_back(pose);
    while (poses_.size() > 500) poses_.pop_front();
    last_odom_ = monotonic(); odom_error_.clear();
  }
  void integrate() {
    if (!pending_ || !fault_.empty()) return;
    const auto cloud = pending_; pending_.reset();
    if (!fresh(cloud->header.stamp)) { cloud_error_ = "cloud_timeout"; return; }
    const int64_t stamp = stamp_ns(cloud->header.stamp);
    if (stamp == map_->last_stamp()) return;  // Duplicate is not a new observation.
    if (stamp < map_->last_stamp()) {
      fault_ = "cloud_timestamp_regressed_reset_required"; publish(); return;
    }
    if (poses_.empty()) { cloud_error_ = "waiting_for_body_pose"; return; }
    const auto closest = std::min_element(poses_.begin(), poses_.end(), [stamp](const Pose& a, const Pose& b) {
      return std::abs(a.stamp-stamp) < std::abs(b.stamp-stamp);
    });
    if (std::abs(closest->stamp-stamp) * 1e-9 > skew_ ||
        monotonic() - last_odom_ > source_timeout_) {
      cloud_error_ = "pose_cloud_skew_or_odom_timeout"; return;
    }
    try {
      std::vector<Field> fields;
      for (const auto& f : cloud->fields) fields.push_back({f.name, f.offset, f.datatype, f.count});
      auto points = decode_xyz(cloud->data.data(), cloud->data.size(), cloud->width,
        cloud->height, cloud->point_step, cloud->row_step, cloud->is_bigendian,
        fields, max_input_points_);
      map_->integrate(points, closest->position, stamp, monotonic());
      last_input_ = monotonic(); cloud_error_.clear();
    } catch (const MapLimitError& error) {
      fault_ = error.what(); publish();
    } catch (const std::exception& error) {
      cloud_error_ = error.what();
    }
  }
  void update_cache() {
    if (cache_version_ == map_->version()) return;
    const auto points = map_->snapshot();
    cached_ = Cloud();
    cached_.height = 1; cached_.width = static_cast<uint32_t>(points.size());
    cached_.point_step = 12; cached_.row_step = 12 * cached_.width;
    cached_.is_bigendian = native_big_endian(); cached_.is_dense = true;
    for (size_t i = 0; i < 3; ++i) {
      sensor_msgs::msg::PointField field;
      field.name = std::string(1, "xyz"[i]); field.offset = 4 * i;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
      cached_.fields.push_back(field);
    }
    cached_.data.resize(cached_.row_step);
    for (size_t i = 0; i < points.size(); ++i) {
      const float coordinates[3] = {static_cast<float>(points[i].x),
        static_cast<float>(points[i].y), static_cast<float>(points[i].z)};
      std::memcpy(cached_.data.data() + 12*i, coordinates, 12);
    }
    cache_version_ = map_->version();
  }
  void publish() {
    const double now = monotonic();
    std::string reason = fault_;
    if (reason.empty()) reason = odom_error_;
    if (reason.empty()) reason = cloud_error_;
    if (reason.empty() && (now-last_input_ > source_timeout_ || now-last_odom_ > source_timeout_))
      reason = "input_timeout_or_waiting_for_input";
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(map_->last_stamp() / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(map_->last_stamp() % 1000000000LL);
    if (reason.empty() && !fresh(stamp)) reason = "input_timestamp_invalid";
    if (reason.empty() && map_->confirmed() < min_points_) reason = "collecting_confirmed_surface";
    Snapshot output;
    output.header.frame_id = frame_; output.header.stamp = stamp;
    output.epoch = epoch_; output.version = map_->version();
    output.healthy = reason.empty(); output.reason = reason.empty() ? "ready" : reason;
    if (output.healthy) {
      update_cache(); output.cloud = cached_; output.cloud.header = output.header;
      if (cloud_pub_->get_subscription_count()) cloud_pub_->publish(output.cloud);
    }
    snapshot_pub_->publish(output);
    // Fixed status reason codes/exception strings are JSON-escaped, not evaluated.
    std::string escaped;
    for (unsigned char c : output.reason) {
      if (c == '"' || c == '\\') escaped.push_back('\\');
      if (c >= 32) escaped.push_back(static_cast<char>(c));
    }
    std::ostringstream stream;
    stream << "{\"healthy\":" << (output.healthy ? "true" : "false")
           << ",\"reason\":\"" << escaped << "\",\"epoch\":\"" << epoch_
           << "\",\"version\":" << map_->version() << ",\"voxels\":" << map_->size()
           << ",\"confirmed_points\":" << map_->confirmed() << "}";
    std_msgs::msg::String status; status.data = stream.str(); status_pub_->publish(status);
  }

  std::unique_ptr<VoxelMap> map_;
  std::string frame_, base_, epoch_, fault_, cloud_error_, odom_error_;
  double source_timeout_, skew_, translation_jump_, rotation_jump_, last_input_, last_odom_;
  size_t max_input_points_, min_points_;
  uint64_t cache_version_ = 0, resets_ = 0;
  std::deque<Pose> poses_;
  Cloud::ConstSharedPtr pending_;
  Cloud cached_;
  rclcpp::Publisher<Snapshot>::SharedPtr snapshot_pub_;
  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  rclcpp::TimerBase::SharedPtr integrate_timer_, publish_timer_;
};
}  // namespace go2_map_maintenance

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_map_maintenance::MapMaintenance>());
  rclcpp::shutdown();
  return 0;
}
