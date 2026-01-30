#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <map>

// Struct to store the filter state correctly (using Quaternions)
struct MarkerFilter {
    cv::Vec3d tvec;        // Translation (Linear average is fine)
    tf2::Quaternion q;     // Rotation (Must use Quaternion)
    bool initialized = false;
};

class RobustAruco : public rclcpp::Node {
public:
    RobustAruco() : Node("robust_aruco_node") {
        this->declare_parameter("marker_size_default", 0.094);
        this->declare_parameter("marker_size_small", 0.0378);
        this->declare_parameter("small_marker_id", 3);
        this->declare_parameter("target_id", 1);
        this->declare_parameter("camera_frame", "camera_optical_frame");
        
        // Lower Alpha = SMOOTHER but slower (0.1 = 10% new, 90% old)
        this->declare_parameter("filter_alpha_small", 0.1); 
        this->declare_parameter("filter_alpha_default", 0.3);

        default_size_ = this->get_parameter("marker_size_default").as_double();
        small_size_ = this->get_parameter("marker_size_small").as_double();
        small_id_ = this->get_parameter("small_marker_id").as_int();
        target_id_ = this->get_parameter("target_id").as_int();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        alpha_small_ = this->get_parameter("filter_alpha_small").as_double();
        alpha_default_ = this->get_parameter("filter_alpha_default").as_double();

        // Best Effort QoS to handle potential frame drops smoothly
        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();

        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", qos_profile, 
            std::bind(&RobustAruco::info_callback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_rect", qos_profile, 
            std::bind(&RobustAruco::image_callback, this, std::placeholders::_1));

        debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco_tracker/debug", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        has_info_ = false;
        RCLCPP_INFO(this->get_logger(), "Robust ArUco: Quaternion Slerp Enabled.");
    }

private:
    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (has_info_) return;
        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        dist_coeffs_ = cv::Mat(1, msg->d.size(), CV_64F);
        for (int i = 0; i < 9; i++) camera_matrix_.at<double>(i / 3, i % 3) = msg->k[i];
        for (size_t i = 0; i < msg->d.size(); i++) dist_coeffs_.at<double>(0, i) = msg->d[i];
        has_info_ = true;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            if (!has_info_) return;

            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            if (cv_ptr->image.empty()) return;

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            
            cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
            detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
            cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
            cv::aruco::ArucoDetector detector(dictionary, detectorParams);
            
            detector.detectMarkers(cv_ptr->image, corners, ids);

            if (!ids.empty()) {
                cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
                
                for (size_t i = 0; i < ids.size(); i++) {
                    int current_id = ids[i];
                    double current_size = (current_id == small_id_) ? small_size_ : default_size_;
                    double alpha = (current_id == small_id_) ? alpha_small_ : alpha_default_;

                    std::vector<cv::Point3f> objPoints;
                    float s = current_size / 2.0f;
                    objPoints.push_back(cv::Point3f(-s, s, 0));
                    objPoints.push_back(cv::Point3f(s, s, 0));
                    objPoints.push_back(cv::Point3f(s, -s, 0));
                    objPoints.push_back(cv::Point3f(-s, -s, 0));

                    cv::Vec3d rvec_raw, tvec_raw;
                    
                    // 1. Solver: IPPE_SQUARE is safest for flat markers
                    cv::solvePnP(objPoints, corners[i], camera_matrix_, dist_coeffs_, rvec_raw, tvec_raw, false, cv::SOLVEPNP_IPPE_SQUARE);

                    // --- FILTERING LOGIC ---
                    if (filters_.find(current_id) == filters_.end()) {
                        filters_[current_id] = MarkerFilter();
                    }

                    // Convert Raw OpenCV Rvec -> TF2 Quaternion
                    tf2::Quaternion q_raw = get_quat_from_rvec(rvec_raw);

                    cv::Vec3d tvec_final;
                    tf2::Quaternion q_final;

                    if (!filters_[current_id].initialized) {
                        tvec_final = tvec_raw;
                        q_final = q_raw;
                        filters_[current_id].initialized = true;
                    } else {
                        // 2. Position Filter (Standard Linear Interpolation)
                        tvec_final = alpha * tvec_raw + (1.0 - alpha) * filters_[current_id].tvec;

                        // 3. Rotation Filter (SLERP - The Fix for "Spinning")
                        // Slerp finds the shortest path between rotations on the sphere
                        q_final = filters_[current_id].q.slerp(q_raw, alpha);
                    }

                    // Update State
                    filters_[current_id].tvec = tvec_final;
                    filters_[current_id].q = q_final;

                    // --- PUBLISH ---
                    // Helper to convert back to rvec for drawing (Optional, simpler to just use q_final for ROS)
                    cv::Vec3d rvec_final; 
                    // (We don't actually need rvec_final for ROS, just for OpenCV drawing. 
                    //  We skip drawing axes to save CPU cycles and complexity here)

                    publish_tf_quat(tvec_final, q_final, msg->header.stamp, current_id);
                    ensure_publisher_exists(current_id);
                    publish_marker_pose(tvec_final, q_final, msg->header, current_id);
                }
            }
            debug_pub_->publish(*cv_ptr->toImageMsg());
        } catch (const std::exception& e) {
             RCLCPP_ERROR(this->get_logger(), "Callback Error: %s", e.what());
        } catch (...) {
             RCLCPP_ERROR(this->get_logger(), "Unknown Error");
        }
    }

    void ensure_publisher_exists(int id) {
        if (marker_pubs_.find(id) == marker_pubs_.end()) {
            std::string topic_name = "/aruco/marker_" + std::to_string(id);
            marker_pubs_[id] = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);
        }
    }

    tf2::Quaternion get_quat_from_rvec(cv::Vec3d rvec) {
        cv::Mat rot_matrix;
        cv::Rodrigues(rvec, rot_matrix);
        tf2::Matrix3x3 tf_rot(
            rot_matrix.at<double>(0,0), rot_matrix.at<double>(0,1), rot_matrix.at<double>(0,2),
            rot_matrix.at<double>(1,0), rot_matrix.at<double>(1,1), rot_matrix.at<double>(1,2),
            rot_matrix.at<double>(2,0), rot_matrix.at<double>(2,1), rot_matrix.at<double>(2,2)
        );
        tf2::Quaternion q;
        tf_rot.getRotation(q);
        return q;
    }

    void publish_tf_quat(cv::Vec3d tvec, tf2::Quaternion q, rclcpp::Time stamp, int id) {
        try {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = stamp;
            t.header.frame_id = camera_frame_;
            t.child_frame_id = "marker_" + std::to_string(id);
            t.transform.translation.x = tvec[0];
            t.transform.translation.y = tvec[1];
            t.transform.translation.z = tvec[2];
            t.transform.rotation.x = q.x(); 
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z(); 
            t.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(t);
        } catch (const std::exception &ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
        }
    }

    void publish_marker_pose(cv::Vec3d tvec, tf2::Quaternion q, std_msgs::msg::Header header, int id) {
        geometry_msgs::msg::PoseStamped p;
        p.header = header;
        p.pose.position.x = tvec[0];
        p.pose.position.y = tvec[1];
        p.pose.position.z = tvec[2];
        p.pose.orientation.x = q.x(); 
        p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z(); 
        p.pose.orientation.w = q.w();
        marker_pubs_[id]->publish(p);
    }

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::map<int, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> marker_pubs_;
    
    // State storage
    std::map<int, MarkerFilter> filters_;

    cv::Mat camera_matrix_, dist_coeffs_;
    bool has_info_;
    double default_size_, small_size_;
    double alpha_small_, alpha_default_;
    int small_id_, target_id_;
    std::string camera_frame_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobustAruco>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}