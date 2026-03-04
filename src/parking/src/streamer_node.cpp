#include <csignal> // Add this include at the very top!
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class VideoStreamer : public rclcpp::Node {
public:
    VideoStreamer() : Node("video_streamer") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/aruco_tracker/debug", 10, std::bind(&VideoStreamer::image_callback, this, std::placeholders::_1));
        // Start HTTP Server Thread
        server_thread_ = std::thread(&VideoStreamer::run_server, this);
        RCLCPP_INFO(this->get_logger(), "MJPEG Streamer started on port 8080 at /stream.mjpg");
    }

    ~VideoStreamer() {
        running_ = false;
        if (server_thread_.joinable()) server_thread_.join();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // Use toCvShare if possible to reduce overhead, or toCvCopy for safety
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        
        std::lock_guard<std::mutex> lock(frame_mutex_);
        current_frame_ = frame; // Update the shared buffer
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}
    void run_server() {
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(8080);

        bind(server_fd, (struct sockaddr*)&address, sizeof(address));
        listen(server_fd, 5);

        while (running_ && rclcpp::ok()) {
            struct sockaddr_in client_addr;
            socklen_t addrlen = sizeof(client_addr);
            int client_socket = accept(server_fd, (struct sockaddr*)&client_addr, &addrlen);

            if (client_socket >= 0) {
                std::thread(&VideoStreamer::handle_client, this, client_socket).detach();
            }
        }
        close(server_fd);
    }

    void handle_client(int socket) {
    // 1. Send the HTTP header immediately so the browser stops "loading"
    std::string response = 
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    
    if (send(socket, response.c_str(), response.length(), 0) < 0) {
        close(socket);
        return;
    }

    while (running_) {
        std::vector<uchar> buf;
        {
            // Simple lock
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (current_frame_.empty()) {
                continue; 
            }
            cv::imencode(".jpg", current_frame_, buf);
        }

        // 2. The MJPEG part
        std::string header = 
            "--frame\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: " + std::to_string(buf.size()) + "\r\n\r\n";
        
        // Use send() with the 0 flag for all 3 parts
        if (send(socket, header.c_str(), header.length(), 0) < 0) break;
        if (send(socket, buf.data(), buf.size(), 0) < 0) break;
        if (send(socket, "\r\n", 2, 0) < 0) break;

        // Slow it down just a tiny bit so we don't overwhelm the network
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    close(socket);
}

    bool running_ = true;
    cv::Mat current_frame_;
    std::mutex frame_mutex_;
    std::thread server_thread_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    std::signal(SIGPIPE, SIG_IGN);

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoStreamer>();
    RCLCPP_INFO(node->get_logger(), "Streamer initialized and ignoring SIGPIPE.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}