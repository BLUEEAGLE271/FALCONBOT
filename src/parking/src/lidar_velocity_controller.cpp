#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <fcntl.h>  // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <unistd.h>  // UNIX standard definitions
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

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

        // --- TIMER (20Hz Control Loop) ---
        timer_ = this->create_wall_timer(
            50ms, std::bind(&LidarVelocityController::control_loop, this));
            
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

    // --- ROS HANDLES ---
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- SERIAL HELPER ---
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

    void control_loop() {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        double error = target_linear_ - current_linear_;
        double output_mps = 0.0; 

        // Deadband (Stop if target is near zero)
        if (std::abs(target_linear_) < 0.01) {
            integral_error_ = 0.0;
            output_mps = 0.0;
        } else {
            // PID Logic
            integral_error_ += error * dt;

            // Anti-Windup Clamp (+/- 0.3 m/s boost)
            if (integral_error_ > 0.3) integral_error_ = 0.3;
            if (integral_error_ < -0.3) integral_error_ = -0.3;

            // Feedforward
            double ff_term = target_linear_; 
            double pid_term = (kp_ * error) + (ki_ * integral_error_);
            
            output_mps = ff_term + pid_term;
        }

        // Clamp Output to Physical Limits
        if (output_mps > max_speed_) output_mps = max_speed_;
        if (output_mps < -max_speed_) output_mps = -max_speed_;

        // --- JSON SERIALIZATION ---
        char buffer[64];
        int len = snprintf(buffer, sizeof(buffer), "{\"T\":5,\"L\":%.3f,\"A\":%.3f}\n", 
                           output_mps, target_angular_);

        if (serial_fd_ != -1) {
            write(serial_fd_, buffer, len);
        }

        // --- ADDED FOR DEBUGGING: SEE WHAT IS HAPPENING ---
        // This will print every 200ms (to avoid spamming too fast)
        static int debug_counter = 0;
        if (debug_counter++ % 4 == 0) {
             RCLCPP_INFO(this->get_logger(), "Tgt: %.2f | Act: %.2f | Out: %.2f", 
                         target_linear_, current_linear_, output_mps);
        }
    }
};

int main(int argc, char **argv)
{
    // --- ADDED FOR DEBUGGING: FIX STUCK LOGS ---
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarVelocityController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}