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
#include <fstream> // Required for file saving
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
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
        this->declare_parameter("serial_port", "/dev/esp32");
        this->declare_parameter("smc_lambda", 2.0);
        this->declare_parameter("smc_k", 0.6);
        this->declare_parameter("smc_phi", 0.15);

        max_speed_ = this->get_parameter("max_linear_speed").as_double();
        smc_lambda_ = this->get_parameter("smc_lambda").as_double();
        smc_k_ = this->get_parameter("smc_k").as_double();
        smc_phi_ = this->get_parameter("smc_phi").as_double();
        load_smc_from_file();
        std::string port = this->get_parameter("serial_port").as_string();

        // --- SERIAL SETUP ---
        if (!setupSerial(port)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port.c_str());
            rclcpp::shutdown();
        }

        // --- SUBSCRIBERS ---
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&LidarVelocityController::cmd_vel_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom_rf2o", 10, std::bind(&LidarVelocityController::odom_callback, this, _1));

        // --- NEW: IMU PUBLISHER ---
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        
        mission_trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>("/mission/trigger", 10);
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
    double current_angular_ = 0.0; // <--- ADD THIS
    double integral_error_ = 0.0;
    double max_speed_;
    double smc_lambda_, smc_k_, smc_phi_; // New SMC Variables
    rclcpp::Time last_time_;
    
    // Buffer for storing incoming serial bytes until we find a newline
    std::string serial_buffer_ = ""; 
    std::string smc_file_ = "/home/blueeagle/lidar_smc_config.txt";
    // --- ROS HANDLES ---
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_trigger_pub_;
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
        current_angular_ = msg->twist.twist.angular.z;
    }

    // --- NEW: PARSE JSON MANUALLY ---
    // Example: {"T":2,"ax":0.123,"gz":0.005}
    void parse_serial_data(std::string line) {
        
        

        // 2. Check for Mission Trigger: {"T":202,"cmd":1}
        if (line.find("\"T\":202") != std::string::npos) {
            try {
                size_t cmd_pos = line.find("\"cmd\":");
                if (cmd_pos != std::string::npos) {
                    char cmd_char = line.at(cmd_pos + 6); // Grab digit after "cmd":
                    auto msg = std_msgs::msg::Bool();
                    msg.data = (cmd_char == '1'); // 1 = Start, 0 = Abort
                    mission_trigger_pub_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "MISSION TRIGGER: %s", msg.data ? "START" : "ABORT");
                }
            } catch (...) {}
        }

        // 3. IMU Data: {"T":2...}
        else if (line.find("\"T\":2") != std::string::npos) {
            try {
                size_t ax_pos = line.find("\"ax\":");
                size_t gz_pos = line.find("\"gz\":");
                size_t end_pos = line.find("}");

                if (ax_pos != std::string::npos && gz_pos != std::string::npos) {
                    std::string ax_str = line.substr(ax_pos + 5, gz_pos - (ax_pos + 5) - 1);
                    std::string gz_str = line.substr(gz_pos + 5, end_pos - (gz_pos + 5));   

                    double ax = std::stod(ax_str);
                    double gz = std::stod(gz_str);

                    auto imu_msg = sensor_msgs::msg::Imu();
                    imu_msg.header.stamp = this->now();
                    imu_msg.header.frame_id = "base_link"; 
                    imu_msg.linear_acceleration.x = ax;
                    imu_msg.angular_velocity.z = gz;
                    
                    // Covariances
                    imu_msg.orientation_covariance[0] = -1; 
                    imu_msg.linear_acceleration_covariance[0] = 0.01;
                    imu_msg.angular_velocity_covariance[0] = 0.01;

                    imu_pub_->publish(imu_msg);
                }
            } catch (...) {}
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
                parse_serial_data(line);
                
                // Remove processed line
                serial_buffer_.erase(0, newline_pos + 1);
            }
        }
    }

    const std::string pid_file_ = "/home/blueeagle/lidar_pid_config.txt";

    void save_smc_to_file() {
        std::ofstream outfile(smc_file_);
        if (outfile.is_open()) {
            outfile << smc_lambda_ << " " << smc_k_ << " " << smc_phi_;
            outfile.close();
            RCLCPP_INFO(this->get_logger(), "SMC Saved: L=%.2f, K=%.2f, Phi=%.2f", smc_lambda_, smc_k_, smc_phi_);
        }
    }

    void load_smc_from_file() {
        std::ifstream infile(smc_file_);
        if (infile.is_open()) {
            if (infile >> smc_lambda_ >> smc_k_ >> smc_phi_) {
                RCLCPP_INFO(this->get_logger(), "Loaded SMC: L=%.2f, K=%.2f, Phi=%.2f", smc_lambda_, smc_k_, smc_phi_);
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

        // --- SMC TUNING PARAMETERS ---
        // (You can later move these to your declare_parameter block)
        ;         // Boundary Layer Thickness (smooths out the 20Hz chatter)

        if (std::abs(target_linear_) < 0.01) {
            integral_error_ = 0.0;
            //output_mps = 0.0;
        } else {
            // 1. Integral of the error (Required for the Sliding Surface)
            integral_error_ += error * dt;
            // Cap it slightly higher to allow for sustained pushing during tight turns
            if (integral_error_ > 1.0) integral_error_ = 1.0;
            if (integral_error_ < -1.0) integral_error_ = -1.0;

            // 2. Define the Sliding Surface (s)
            double s = error + (smc_lambda_ * integral_error_);

            // 3. The Saturation Function (Creates the Boundary Layer)
            double sat_s = s / smc_phi_;
            if (sat_s > 1.0) sat_s = 1.0;
            if (sat_s < -1.0) sat_s = -1.0;

            // 4. Equivalent Control (Feedforward) + Switching Control
            double u_eq = target_linear_; 
            
            // Dynamic Turn Compensation: Add more power if we are turning
            double turn_friction_boost = std::abs(target_angular_) * 0.05;
            u_eq += (target_linear_ > 0 ? turn_friction_boost : -turn_friction_boost);

            // Final Output Calculation
            output_mps = u_eq + (smc_k_ * sat_s);
        }

        // Hardware safety limits
        if (output_mps > max_speed_) output_mps = max_speed_;
        if (output_mps < -max_speed_) output_mps = -max_speed_;

        // DEBUG PRINT: Placed AFTER math to show the true output
        /* RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 50, 
            "LOG | L_Tgt: %.2f L_Meas: %.2f | A_Tgt: %.2f A_Meas: %.2f | OUT: %.2f", 
            target_linear_, current_linear_, target_angular_, current_angular_, output_mps);
        // --- CALL TO ACTION ---
        // Triggers when the SMC successfully forces the robot to the target speed
        if (std::abs(error) < 0.05 && std::abs(target_linear_) > 0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                ">>> TARGET VELOCITY LOCKED: READY TO COMMENCE PARKING SEQUENCE! <<<");
        } */

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