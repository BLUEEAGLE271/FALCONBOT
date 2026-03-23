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
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp" // NEW: Required for IMU
#include <nlohmann/json.hpp>

using json = nlohmann::json;
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
        this->declare_parameter("kp_v", 7.0);
        this->declare_parameter("ki_v", 0.0);
        this->declare_parameter("kd_v", 0.0);
        this->declare_parameter("kp_w", 3.0);
        this->declare_parameter("ki_w", 0.0);
        this->declare_parameter("kd_w", 0.0);

        update_pid_params();

        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto &param : parameters) {
                    if (param.get_name() == "kp_v") kp_v_ = param.as_double();
                    if (param.get_name() == "ki_v") ki_v_ = param.as_double();
                    if (param.get_name() == "kd_v") kd_v_ = param.as_double();
                    if (param.get_name() == "kp_w") kp_w_ = param.as_double();
                    if (param.get_name() == "ki_w") ki_w_ = param.as_double();
                    if (param.get_name() == "kd_w") kd_w_ = param.as_double();
                }
                return result;
            });
        
        

        max_speed_ = this->get_parameter("max_linear_speed").as_double();
        smc_lambda_ = this->get_parameter("smc_lambda").as_double();
        smc_k_ = this->get_parameter("smc_k").as_double();
        smc_phi_ = this->get_parameter("smc_phi").as_double();
        load_smc_from_file();
        std::string port = this->get_parameter("serial_port").as_string();

        load_nn_weights("/home/blueeagle/ros2_ws/nn_weights.json");

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
        raw_serial_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/esp32_write", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
                if (serial_fd_ != -1) {
                    write(serial_fd_, msg->data.c_str(), msg->data.length());
                    write(serial_fd_, "\n", 1); // Ensure newline termination
                }
            });
        // --- NEW: IMU PUBLISHER ---
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        pwm_pub_ = this->create_publisher<std_msgs::msg::String>("/motor_pwm", 10);
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
    double kp_v_, ki_v_, kd_v_;
    double kp_w_, ki_w_, kd_w_;
    double integral_v_ = 0.0, prev_error_v_ = 0.0;
    double integral_w_ = 0.0, prev_error_w_ = 0.0;
    double pwm_left_prev_  = 0.0;   // ← add
    double pwm_right_prev_ = 0.0;   // ← add
    double max_pwm_rate_   = 10.0;  // ← add — max PWM change per 50ms cycle
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rclcpp::Time last_time_;

    //Neutral network
    std::vector<std::vector<double>> w1_, w2_, w3_;
    std::vector<double> b1_, b2_, b3_;
    
    // Buffer for storing incoming serial bytes until we find a newline
    std::string serial_buffer_ = ""; 
    std::string smc_file_ = "/home/blueeagle/lidar_smc_config.txt";
    // --- ROS HANDLES ---
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr raw_serial_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_trigger_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr read_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pwm_pub_;

    void update_pid_params() {
        kp_v_ = this->get_parameter("kp_v").as_double();
        ki_v_ = this->get_parameter("ki_v").as_double();
        kd_v_ = this->get_parameter("kd_v").as_double();
        kp_w_ = this->get_parameter("kp_w").as_double();
        ki_w_ = this->get_parameter("ki_w").as_double();
        kd_w_ = this->get_parameter("kd_w").as_double();
    }

    std::vector<double> relu(const std::vector<double>& x) {
        std::vector<double> out = x;
        for (auto& val : out) if (val < 0.0) val = 0.0;
        return out;
    }

    std::vector<double> matmul_add(const std::vector<std::vector<double>>& W, const std::vector<double>& x, const std::vector<double>& b) {
        std::vector<double> out(b.size(), 0.0);
        for (size_t i = 0; i < W.size(); ++i) {
            for (size_t j = 0; j < x.size(); ++j) {
                out[i] += W[i][j] * x[j];
            }
            out[i] += b[i];
        }
        return out;
    }

    void load_nn_weights(const std::string& filepath) {
        std::ifstream f(filepath);
        if (!f.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "CRITICAL: Could not find nn_weights.json!");
            return;
        }
        json data = json::parse(f);
        w1_ = data["w1"].get<std::vector<std::vector<double>>>();
        b1_ = data["b1"].get<std::vector<double>>();
        w2_ = data["w2"].get<std::vector<std::vector<double>>>();
        b2_ = data["b2"].get<std::vector<double>>();
        w3_ = data["w3"].get<std::vector<std::vector<double>>>();
        b3_ = data["b3"].get<std::vector<double>>();
        RCLCPP_INFO(this->get_logger(), "Neural Network Weights Loaded Successfully!");
    }

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

        double final_pwm_left  = 0.0;
        double final_pwm_right = 0.0;

        if (std::abs(target_linear_) < 0.01 && std::abs(target_angular_) < 0.01) {
            // Reset integrators on stop
            integral_v_ = 0.0;
            integral_w_ = 0.0;
            prev_error_v_ = 0.0;
            prev_error_w_ = 0.0;
            pwm_left_prev_  = 0.0;   // ← add
            pwm_right_prev_ = 0.0;   // ← add  
            // final_pwm stays 0 — stop command will be sent below
        } else {
            // 1. NN generates base PWM from raw target velocity
            std::vector<double> inputs = {target_linear_, target_angular_};
            auto h1 = relu(matmul_add(w1_, inputs, b1_));
            auto h2 = relu(matmul_add(w2_, h1, b2_));
            auto y  = matmul_add(w3_, h2, b3_);

            double base_pwm_left  = y[0] * 250.0;
            double base_pwm_right = y[1] * 250.0;

            // 2. PID trims the PWM output directly — gains in PWM/velocity units
            // kp_v=50 means: 1 m/s error → add 50 PWM correction
            double error_v = target_linear_ - current_linear_;
            integral_v_ += error_v * dt;
            integral_v_ = std::clamp(integral_v_, -50.0 / ki_v_, 50.0 / ki_v_);
            double deriv_v = (error_v - prev_error_v_) / dt;
            double pid_v = (kp_v_ * error_v)
                        + (ki_v_ * integral_v_)
                        + (kd_v_ * deriv_v);   // output is in PWM units
            prev_error_v_ = error_v;

            double error_w = target_angular_ - current_angular_;
            integral_w_ += error_w * dt;
            integral_w_ = std::clamp(integral_w_, -50.0 / ki_w_, 50.0 / ki_w_);
            double deriv_w = (error_w - prev_error_w_) / dt;
            double pid_w = (kp_w_ * error_w)
                        + (ki_w_ * integral_w_)
                        + (kd_w_ * deriv_w);   // output is in PWM units
            prev_error_w_ = error_w;

            // 3. Add PID trim to NN base PWM
            final_pwm_left  = base_pwm_left  + pid_v - pid_w;
            final_pwm_right = base_pwm_right + pid_v + pid_w;

            final_pwm_left  = std::clamp(final_pwm_left,  -255.0, 255.0);
            final_pwm_right = std::clamp(final_pwm_right, -255.0, 255.0);

            
        }

        // Serial write and PWM publish are OUTSIDE if/else
        // This guarantees the stop command (PWM=0) is always sent
        char buffer[64];
        int len = snprintf(buffer, sizeof(buffer), "{\"T\":11,\"L\":%.0f,\"R\":%.0f}\n",
                        final_pwm_left, final_pwm_right);

        static bool last_was_zero = false;
        bool current_is_zero = (std::abs(final_pwm_left) < 1.0 &&
                                std::abs(final_pwm_right) < 1.0);

        if (serial_fd_ != -1 && (!current_is_zero || !last_was_zero)) {
            write(serial_fd_, buffer, len);
            std_msgs::msg::String pwm_msg;
            pwm_msg.data = std::to_string((int)final_pwm_left) + ","
                        + std::to_string((int)final_pwm_right);
            pwm_pub_->publish(pwm_msg);
        }
        last_was_zero = current_is_zero;
    }  // ← closing brace for control_loop
    
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