#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
// JetPack 6 / OpenCV 4.8 Header
#include <opencv2/objdetect/aruco_detector.hpp>

class RobustAruco : public rclcpp::Node {
public:
    RobustAruco() : Node("robust_aruco_node") {
        this->declare_parameter("marker_size_default", 0.094);
        this->declare_parameter("marker_size_small", 0.0378);
        this->declare_parameter("small_marker_id", 3);
        this->declare_parameter("target_id", 1);
        this->declare_parameter("camera_frame", "camera_optical_frame");
        this->declare_parameter("marker_frame_prefix", "marker_");
        this->declare_parameter("filter_alpha", 0.3);

        default_size_ = this->get_parameter("marker_size_default").as_double();
        small_size_ = this->get_parameter("marker_size_small").as_double();
        small_id_ = this->get_parameter("small_marker_id").as_int();
        target_id_ = this->get_parameter("target_id").as_int();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        marker_prefix_ = this->get_parameter("marker_frame_prefix").as_string();
        filter_alpha_ = this->get_parameter("filter_alpha").as_double();

        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", qos_profile, 
            std::bind(&RobustAruco::info_callback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_rect", qos_profile, 
            std::bind(&RobustAruco::image_callback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/parking_goal", 10);
        debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco_tracker/debug", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        has_info_ = false;
        has_prev_pose_ = false;
        RCLCPP_INFO(this->get_logger(), "Robust ArUco (Jetson OpenCV 4.8) initialized.");
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
        if (!has_info_) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            // ZERO COPY IS NOW SAFE! 
            // Since we are compiling cv_bridge locally, it matches the system version.
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) { return; }

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

                std::vector<cv::Point3f> objPoints;
                float s = current_size / 2.0f;
                objPoints.push_back(cv::Point3f(-s, s, 0));
                objPoints.push_back(cv::Point3f(s, s, 0));
                objPoints.push_back(cv::Point3f(s, -s, 0));
                objPoints.push_back(cv::Point3f(-s, -s, 0));

                cv::Vec3d rvec, tvec;
                cv::solvePnP(objPoints, corners[i], camera_matrix_, dist_coeffs_, rvec, tvec);

                publish_tf(tvec, rvec, msg->header.stamp, current_id);

                if (current_id == target_id_) {
                    if (has_prev_pose_) {
                        prev_t_ = filter_alpha_ * tvec + (1.0 - filter_alpha_) * prev_t_;
                        prev_r_ = filter_alpha_ * rvec + (1.0 - filter_alpha_) * prev_r_;
                    } else {
                        prev_t_ = tvec;
                        prev_r_ = rvec;
                        has_prev_pose_ = true;
                    }
                    publish_pose(prev_t_, prev_r_, msg->header);
                    cv::drawFrameAxes(cv_ptr->image, camera_matrix_, dist_coeffs_, prev_r_, prev_t_, 0.05);
                }
            }
        }
        debug_pub_->publish(*cv_ptr->toImageMsg());
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

    void publish_tf(cv::Vec3d tvec, cv::Vec3d rvec, rclcpp::Time stamp, int id) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = camera_frame_;
        t.child_frame_id = marker_prefix_ + std::to_string(id);
        t.transform.translation.x = tvec[0];
        t.transform.translation.y = tvec[1];
        t.transform.translation.z = tvec[2];
        tf2::Quaternion q = get_quat_from_rvec(rvec);
        t.transform.rotation.x = q.x(); t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z(); t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
    }

    void publish_pose(cv::Vec3d tvec, cv::Vec3d rvec, std_msgs::msg::Header header) {
        geometry_msgs::msg::PoseStamped p;
        p.header = header;
        p.pose.position.x = tvec[0];
        p.pose.position.y = tvec[1];
        p.pose.position.z = tvec[2];
        tf2::Quaternion q = get_quat_from_rvec(rvec);
        p.pose.orientation.x = q.x(); p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z(); p.pose.orientation.w = q.w();
        pose_pub_->publish(p);
    }

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    cv::Mat camera_matrix_, dist_coeffs_;
    bool has_info_, has_prev_pose_;
    cv::Vec3d prev_t_, prev_r_;
    double default_size_, small_size_, filter_alpha_;
    int small_id_, target_id_;
    std::string camera_frame_, marker_prefix_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobustAruco>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}