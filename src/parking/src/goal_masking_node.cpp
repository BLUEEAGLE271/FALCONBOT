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
    this->declare_parameter("mask_radius", 0.25);
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
    //RCLCPP_INFO(this->get_logger(), "New Goal Received: [%.2f, %.2f]", 
     // msg->pose.position.x, msg->pose.position.y);
  }

  void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    // Passthrough if no goal received yet
    if (!current_goal_) {
      scan_pub_->publish(*scan_msg);
      return;
    }

    // --- 1. ROBUST TF (Fixes "Extrapolation into the past") ---
    geometry_msgs::msg::PoseStamped local_goal;
    try {
      // Use TimePointZero to get the most recent transform available
      // This ignores the timestamp difference between the Goal and the Scan
      geometry_msgs::msg::PoseStamped goal_latest = *current_goal_;
      goal_latest.header.stamp = rclcpp::Time(0); 

      tf_buffer_->transform(goal_latest, local_goal, scan_msg->header.frame_id, 
                            tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & ex) {
      // Only warn occasionally to keep logs clean
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF Fail: %s", ex.what());
      scan_pub_->publish(*scan_msg);
      return;
    }

    // --- 2. THE NUCLEAR OPTION (Circle Mask) ---
    // Since walls are 3mm, we CANNOT rely on the map resolution.
    // We must clear the entire area (walls included) and trust the Local Planner.
    double goal_x = local_goal.pose.position.x;
    double goal_y = local_goal.pose.position.y;
    double mask_radius_sq = 0.25 * 0.25; // 0.4m Radius = 0.8m Hole

    auto masked_scan = std::make_unique<sensor_msgs::msg::LaserScan>(*scan_msg);

    // Optimized filtering loop
    double angle = scan_msg->angle_min;
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        float r = scan_msg->ranges[i];
        if (std::isfinite(r)) {
            // Polar to Cartesian
            double px = r * cos(angle);
            double py = r * sin(angle);

            // Distance Check
            double dist_sq = (px - goal_x)*(px - goal_x) + (py - goal_y)*(py - goal_y);
            
            if (dist_sq < mask_radius_sq) {
                // DELETE THE POINT
                masked_scan->ranges[i] = std::numeric_limits<float>::infinity();
            }
        }
        angle += scan_msg->angle_increment;
    }

    scan_pub_->publish(std::move(masked_scan));
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