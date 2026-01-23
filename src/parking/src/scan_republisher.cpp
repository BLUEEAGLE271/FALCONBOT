#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <algorithm> // For std::rotate

class ScanRepublisher : public rclcpp::Node
{
public:
    ScanRepublisher() : Node("scan_republisher")
    {
        // 1. Subscribe (Best Effort to match Lidar)
        auto qos_policy = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            qos_policy, 
            std::bind(&ScanRepublisher::topic_callback, this, std::placeholders::_1));

        // 2. Publish (Reliable for rf2o)
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_nav", 10);
        
        RCLCPP_INFO(this->get_logger(), "C++ Scan Republisher Started (High Performance)");
    }

private:
    void topic_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
    {
        // 3. Frame Override
        msg->header.frame_id = "base_laser_nav";
        
        // 4. Array Rotation logic
        // We want to shift the data by +90 degrees (Quarter of the array)
        // Python equivalent: ranges = ranges[shift:] + ranges[:shift]
        
        size_t count = msg->ranges.size();
        if (count > 0) {
            size_t shift = count / 4; // 90 degrees

            // std::rotate is extremely optimized. 
            // It moves the element at 'begin + shift' to 'begin'.
            std::rotate(msg->ranges.begin(), msg->ranges.begin() + shift, msg->ranges.end());
            
            // Do the same for intensities if they exist
            if (msg->intensities.size() == count) {
                std::rotate(msg->intensities.begin(), msg->intensities.begin() + shift, msg->intensities.end());
            }
        }

        // 5. Publish (Zero-Copy)
        publisher_->publish(std::move(msg));
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanRepublisher>());
    rclcpp::shutdown();
    return 0;
}