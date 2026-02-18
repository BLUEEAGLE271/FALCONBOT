#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <fcntl.h>  
#include <termios.h> 
#include <unistd.h>  
#include <iostream>
#include <sstream> // For parsing

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp" // NEW: Required for IMU

using namespace std::chrono_literals;
using std::placeholders::_1;

class LidarVelocityController : public rclcpp::Node
{
public:
    LidarVelocityController() : Node("lidar_velocity_controller")
    {
        // --- TUNING PARAMETERS ---
        this->declare_parameter("max_linear_speed", 1.0);
        this->declare_parameter("kp", 0.5);
        this->declare_parameter("ki", 0.2);
        this->declare_parameter("serial_port", "/dev/ttyTHS1");

        max_speed_ = this->get_parameter("max_linear_speed").as_double();
        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        std::string port = this->get_parameter("serial_port").as_string();

        // --- SERIAL SETUP ---
        if (!setupSerial(port)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port.c_str());
            rclcpp::shutdown();
        }

        // --- SUBSCRIBERS ---
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&LidarVelocityController::cmd_vel_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom_rf2o", 10, std::bind(&LidarVelocityController::odom_callback, this, _1));

        // --- NEW: IMU PUBLISHER ---
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        // --- TIMERS ---
        // Timer 1: Control Loop (Write Commands) - 20Hz
        control_timer_ = this->create_wall_timer(
            50ms, std::bind(&LidarVelocityController::control_loop, this));
            
        // Timer 2: Serial Read Loop (Read IMU) - 100Hz (Fast reading)
        read_timer_ = this->create_wall_timer(
            10ms, std::bind(&LidarVelocityController::read_serial_loop, this));

        last_time_ = this->now();
    }

    ~LidarVelocityController() {
        if (serial_fd_ != -1) close(serial_fd_);
    }

private:
    // --- VARIABLES ---
    int serial_fd_ = -1;
    double target_linear_ = 0.0;
    double target_angular_ = 0.0;
    double current_linear_ = 0.0;
    double integral_error_ = 0.0;
    double max_speed_;
    double kp_, ki_;
    rclcpp::Time last_time_;
    
    // Buffer for storing incoming serial bytes until we find a newline
    std::string serial_buffer_ = ""; 

    // --- ROS HANDLES ---
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr read_timer_;

    // --- SERIAL SETUP ---
    bool setupSerial(const std::string &port) {
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) return false;

        struct termios options;
        tcgetattr(serial_fd_, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD); 
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;              
        options.c_cflag &= ~PARENB;          
        options.c_cflag &= ~CSTOPB;          
        options.c_cflag &= ~CRTSCTS;         
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
        options.c_oflag &= ~OPOST;
        tcsetattr(serial_fd_, TCSANOW, &options);
        return true;
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_linear_ = msg->linear.x;
        target_angular_ = msg->angular.z;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_linear_ = msg->twist.twist.linear.x;
    }

    // --- NEW: PARSE JSON MANUALLY ---
    // Example: {"T":2,"ax":0.123,"gz":0.005}
    void parse_and_publish_imu(std::string line) {
        // Simple manual parsing to avoid heavy libraries
        size_t type_pos = line.find("\"T\":2");
        if (type_pos == std::string::npos) return; // Not an IMU message

        try {
            // Find "ax":
            size_t ax_pos = line.find("\"ax\":");
            size_t gz_pos = line.find("\"gz\":");
            size_t end_pos = line.find("}");

            if (ax_pos != std::string::npos && gz_pos != std::string::npos) {
                // Extract numbers
                std::string ax_str = line.substr(ax_pos + 5, gz_pos - (ax_pos + 5) - 1); // between "ax": and ,
                std::string gz_str = line.substr(gz_pos + 5, end_pos - (gz_pos + 5));   // between "gz": and }

                double ax = std::stod(ax_str);
                double gz = std::stod(gz_str);

                // Publish to ROS
                auto imu_msg = sensor_msgs::msg::Imu();
                imu_msg.header.stamp = this->now();
                imu_msg.header.frame_id = "base_link"; // IMU frame

                // EKF needs ax (Linear Accel)
                imu_msg.linear_acceleration.x = ax;
                
                // EKF needs gz (Angular Vel)
                imu_msg.angular_velocity.z = gz;

                // Set covariances (Signals EKF to trust these values but ignore others)
                imu_msg.orientation_covariance[0] = -1; // No orientation (use rf2o)
                imu_msg.linear_acceleration_covariance[0] = 0.01;
                imu_msg.angular_velocity_covariance[0] = 0.01;

                imu_pub_->publish(imu_msg);
            }
        } catch (...) {
            // Ignore parsing errors
        }
    }

    // --- NEW: READ LOOP ---
    void read_serial_loop() {
        if (serial_fd_ == -1) return;

        char buf[256];
        int n = read(serial_fd_, buf, sizeof(buf) - 1);
        
        if (n > 0) {
            buf[n] = 0; // Null terminate
            serial_buffer_ += buf; // Append to global buffer

            // Process all complete lines in the buffer
            size_t newline_pos;
            while ((newline_pos = serial_buffer_.find('\n')) != std::string::npos) {
                std::string line = serial_buffer_.substr(0, newline_pos);
                parse_and_publish_imu(line);
                
                // Remove processed line
                serial_buffer_.erase(0, newline_pos + 1);
            }
        }
    }

    // --- WRITE LOOP ---
    void control_loop() {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        double error = target_linear_ - current_linear_;
        double output_mps = 0.0; 

        if (std::abs(target_linear_) < 0.01) {
            integral_error_ = 0.0;
            output_mps = 0.0;
        } else {
            integral_error_ += error * dt;
            if (integral_error_ > 0.3) integral_error_ = 0.3;
            if (integral_error_ < -0.3) integral_error_ = -0.3;

            double ff_term = target_linear_; 
            double pid_term = (kp_ * error) + (ki_ * integral_error_);
            output_mps = ff_term + pid_term;
        }

        if (output_mps > max_speed_) output_mps = max_speed_;
        if (output_mps < -max_speed_) output_mps = -max_speed_;

        char buffer[64];
        int len = snprintf(buffer, sizeof(buffer), "{\"T\":5,\"L\":%.3f,\"A\":%.3f}\n", 
                           output_mps, target_angular_);

        if (serial_fd_ != -1) {
            write(serial_fd_, buffer, len);
        }
    }
};

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarVelocityController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}