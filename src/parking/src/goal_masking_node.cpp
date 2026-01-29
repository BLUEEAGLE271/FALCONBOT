#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class GoalMaskingNode : public rclcpp::Node
{
public:
  GoalMaskingNode()
  : Node("goal_masking_node")
  {
    // --- Parameters ---
    // Radius around the goal to clear (meters). 
    // 0.35m covers the 20cm container + margin
    this->declare_parameter("mask_radius", 0.4);
    mask_radius_ = this->get_parameter("mask_radius").as_double();
    mask_radius_sq_ = mask_radius_ * mask_radius_; // Optimization: compare squared distances

    // --- Subscribers ---
    // QoS: Best effort or Reliable (Scan usually Best Effort, Goal is Reliable)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos, std::bind(&GoalMaskingNode::scan_callback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/parking_goal", 10, std::bind(&GoalMaskingNode::goal_callback, this, std::placeholders::_1));

    // --- Publishers ---
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_masked", qos);

    // --- TF2 Setup ---
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Goal Masking Node Started (C++). Radius: %.2f", mask_radius_);
  }

private:
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_goal_ = msg;
    RCLCPP_INFO(this->get_logger(), "New Goal Received: [%.2f, %.2f]", 
      msg->pose.position.x, msg->pose.position.y);
  }

  void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    // Passthrough if no goal received yet
    if (!current_goal_) {
      scan_pub_->publish(*scan_msg);
      return;
    }

    try {
      // 1. Transform Goal (Map Frame) -> Lidar Frame
      // We use a timeout of 0.05s to wait for TF if slightly delayed, but fail fast to keep scan rate high
      geometry_msgs::msg::PoseStamped local_goal;
      
      // Note: We transform to the frame ID in the scan header (e.g., "base_laser")
      tf_buffer_->transform(*current_goal_, local_goal, scan_msg->header.frame_id, 
                            tf2::durationFromSec(0.05));

      double goal_x = local_goal.pose.position.x;
      double goal_y = local_goal.pose.position.y;

      // 2. Iterate and Filter
      // We modify the message in-place if possible, but SharedPtr is const, so we copy.
      auto masked_scan = std::make_unique<sensor_msgs::msg::LaserScan>(*scan_msg);

      double angle = scan_msg->angle_min;
      size_t count = scan_msg->ranges.size();

      for (size_t i = 0; i < count; ++i) {
        float r = scan_msg->ranges[i];

        // Skip invalid points
        if (r < scan_msg->range_min || r > scan_msg->range_max || std::isinf(r) || std::isnan(r)) {
          angle += scan_msg->angle_increment;
          continue;
        }

        // Polar -> Cartesian (Relative to Lidar)
        double px = r * std::cos(angle);
        double py = r * std::sin(angle);

        // Distance Check (using Squared Distance for speed)
        // dist_sq = (px - gx)^2 + (py - gy)^2
        double dist_sq = (px - goal_x) * (px - goal_x) + (py - goal_y) * (py - goal_y);

        if (dist_sq < mask_radius_sq_) {
            // Mask it! Set to infinity (cleared space)
            masked_scan->ranges[i] = std::numeric_limits<float>::infinity();
        }

        angle += scan_msg->angle_increment;
      }

      // 3. Publish
      scan_pub_->publish(std::move(masked_scan));

    } catch (const tf2::TransformException & ex) {
      // If TF fails, fallback to safety: publish raw scan
      // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF Error: %s", ex.what());
      scan_pub_->publish(*scan_msg);
    }
  }

  // Variables
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  
  geometry_msgs::msg::PoseStamped::SharedPtr current_goal_;
  
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double mask_radius_;
  double mask_radius_sq_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalMaskingNode>());
  rclcpp::shutdown();
  return 0;
}