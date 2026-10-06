// Gazebo system plugin: reduces the six distance sensors of the vehicle
// (single gpu_lidar grids, one per side) to the closest obstacle per side and
// publishes them in ONE ROS message (muav_gcs_interfaces/msg/ObstacleInfo).
//
// The lidar topics stay inside Gazebo (gz-transport); nothing is bridged.
//
// SDF usage (inside the <model>):
//   <plugin filename="obstacle_info_plugin" name="muav_gcs_gz::ObstacleInfoPlugin">
//     <ros_namespace>uav_1</ros_namespace>   <!-- optional -->
//     <topic>obstacle_info</topic>           <!-- optional, default obstacle_info -->
//     <frame_id>base_link</frame_id>         <!-- optional, default base_link -->
//     <update_rate>10</update_rate>          <!-- optional (Hz), default 10 -->
//   </plugin>
// The lidars must be named distance_sensor_<dir>_link / distance_<dir> with
// dir in {forward, backward, left, right, up, down}.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>

#include <muav_gcs_interfaces/msg/obstacle_info.hpp>
#include <rclcpp/rclcpp.hpp>

namespace muav_gcs_gz
{

class ObstacleInfoPlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &) override
  {
    const std::string model_name = gz::sim::Model(entity).Name(ecm);
    const std::string world_name =
      gz::sim::World(gz::sim::worldEntity(ecm)).Name(ecm).value_or("default");

    const std::string ros_ns = sdf->Get<std::string>("ros_namespace", "").first;
    const std::string topic = sdf->Get<std::string>("topic", "obstacle_info").first;
    frame_id_ = sdf->Get<std::string>("frame_id", "base_link").first;
    const double rate = sdf->Get<double>("update_rate", 10.0).first;
    if (rate <= 0.0) {
      gzerr << "[ObstacleInfoPlugin] update_rate must be > 0, got " << rate << std::endl;
      return;
    }
    period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / rate));

    // Until the first scan arrives report "nothing detected", not distance 0.
    for (auto & side : sides_) {
      side.detected = false;
      side.distance = std::numeric_limits<float>::infinity();
    }

    // All vehicles share the gz server process, hence the ROS global context.
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    std::string node_name = "obstacle_info_" + model_name;
    std::replace_if(
      node_name.begin(), node_name.end(),
      [](unsigned char c) {return !std::isalnum(c) && c != '_';}, '_');

    // Do not inherit the gz process' command line, and keep the node minimal.
    auto options = rclcpp::NodeOptions()
      .use_global_arguments(false)
      .start_parameter_services(false);
    ros_node_ = std::make_shared<rclcpp::Node>(node_name, ros_ns, options);
    pub_ = ros_node_->create_publisher<muav_gcs_interfaces::msg::ObstacleInfo>(
      topic, rclcpp::SensorDataQoS());

    for (std::size_t i = 0; i < kSides.size(); ++i) {
      const std::string dir = kSides[i].gz_name;
      const std::string scan_topic = "/world/" + world_name + "/model/" + model_name +
        "/link/distance_sensor_" + dir + "_link/sensor/distance_" + dir + "/scan";
      const bool ok = gz_node_.Subscribe<gz::msgs::LaserScan>(
        scan_topic,
        [this, i](const gz::msgs::LaserScan & msg) {OnScan(i, msg);});
      if (!ok) {
        gzerr << "[ObstacleInfoPlugin] failed to subscribe to " << scan_topic << std::endl;
      }
    }

    gzmsg << "[ObstacleInfoPlugin] " << model_name << " -> "
          << pub_->get_topic_name() << " @ " << rate << " Hz" << std::endl;
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager &) override
  {
    if (!pub_ || info.paused) {
      return;
    }
    // simTime goes backwards on a world reset: restart the period.
    if (have_published_ && info.simTime >= last_pub_ && info.simTime - last_pub_ < period_) {
      return;
    }
    have_published_ = true;
    last_pub_ = info.simTime;

    muav_gcs_interfaces::msg::ObstacleInfo out;
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(info.simTime);
    const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime - sec);
    out.header.stamp.sec = static_cast<int32_t>(sec.count());
    out.header.stamp.nanosec = static_cast<uint32_t>(nsec.count());
    out.header.frame_id = frame_id_;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      out.front = sides_[0];
      out.back = sides_[1];
      out.left = sides_[2];
      out.right = sides_[3];
      out.up = sides_[4];
      out.down = sides_[5];
    }
    pub_->publish(out);
  }

private:
  struct Side
  {
    const char * gz_name;  // sensor name suffix in the SDF
  };
  // Order matters: front, back, left, right, up, down (see PostUpdate).
  static constexpr std::array<Side, 6> kSides{{
    {"forward"}, {"backward"}, {"left"}, {"right"}, {"up"}, {"down"}}};

  // gz-transport thread: reduce one scan to its closest obstacle.
  void OnScan(std::size_t side, const gz::msgs::LaserScan & msg)
  {
    constexpr float kInf = std::numeric_limits<float>::infinity();
    const float rmin = static_cast<float>(msg.range_min());
    const float rmax = static_cast<float>(msg.range_max());

    float best = kInf;
    bool too_close = false;
    for (int k = 0; k < msg.ranges_size(); ++k) {
      const double r = msg.ranges(k);
      if (std::isnan(r)) {
        continue;  // erroneous measurement
      }
      if (r == -std::numeric_limits<double>::infinity() || r < rmin) {
        too_close = true;  // closer than the sensor can measure
        continue;
      }
      if (!std::isfinite(r) || r > rmax) {
        continue;  // no return within range
      }
      best = std::min(best, static_cast<float>(r));
    }

    muav_gcs_interfaces::msg::ObstaclePosition pos;
    if (too_close) {
      pos.detected = true;
      pos.distance = rmin;
    } else if (std::isfinite(best)) {
      pos.detected = true;
      pos.distance = best;
    } else {
      pos.detected = false;
      pos.distance = kInf;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    sides_[side] = pos;
  }

  gz::transport::Node gz_node_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Publisher<muav_gcs_interfaces::msg::ObstacleInfo>::SharedPtr pub_;

  std::string frame_id_;
  std::chrono::steady_clock::duration period_{};
  std::chrono::steady_clock::duration last_pub_{};
  bool have_published_{false};

  std::mutex mutex_;
  std::array<muav_gcs_interfaces::msg::ObstaclePosition, 6> sides_;
};

}  // namespace muav_gcs_gz

GZ_ADD_PLUGIN(
  muav_gcs_gz::ObstacleInfoPlugin,
  gz::sim::System,
  muav_gcs_gz::ObstacleInfoPlugin::ISystemConfigure,
  muav_gcs_gz::ObstacleInfoPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(muav_gcs_gz::ObstacleInfoPlugin, "muav_gcs_gz::ObstacleInfoPlugin")
