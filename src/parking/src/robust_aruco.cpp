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
#include <cmath>

struct MarkerFilter {
    cv::Vec3d tvec;
    tf2::Quaternion q;
    bool initialized = false;
    rclcpp::Time last_seen;   // track when we last saw this marker
};

class RobustAruco : public rclcpp::Node {
public:
    RobustAruco() : Node("robust_aruco_node") {
        // Give ArUco 3 cores — leaves 3 for Nav2/SLAM/EKF
        cv::setNumThreads(3);
        cv::setUseOptimized(true);

        this->declare_parameter("marker_size_default", 0.094);
        this->declare_parameter("marker_size_small", 0.0378);
        this->declare_parameter("small_marker_id", 3);
        this->declare_parameter("target_id", 1);
        this->declare_parameter("camera_frame", "camera_optical_frame");
        this->declare_parameter("filter_alpha_small", 0.1);
        this->declare_parameter("filter_alpha_default", 0.3);
        this->declare_parameter("process_every_n_frames", 2);

        // How long (seconds) marker can be unseen before filter resets
        this->declare_parameter("filter_reset_timeout", 0.5);

        // Max rotation change (degrees) per frame before rejecting as a flip
        this->declare_parameter("flip_rejection_threshold_deg", 90.0);

        default_size_  = this->get_parameter("marker_size_default").as_double();
        small_size_    = this->get_parameter("marker_size_small").as_double();
        small_id_      = this->get_parameter("small_marker_id").as_int();
        target_id_     = this->get_parameter("target_id").as_int();
        camera_frame_  = this->get_parameter("camera_frame").as_string();
        alpha_small_   = this->get_parameter("filter_alpha_small").as_double();
        alpha_default_ = this->get_parameter("filter_alpha_default").as_double();
        process_every_n_frames_ = this->get_parameter("process_every_n_frames").as_int();
        filter_reset_timeout_   = this->get_parameter("filter_reset_timeout").as_double();
        flip_threshold_rad_     = this->get_parameter("flip_rejection_threshold_deg").as_double()
                                  * M_PI / 180.0;

        rclcpp::QoS qos_profile = rclcpp::QoS(10).reliable();
        

        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", qos_profile,
            std::bind(&RobustAruco::info_callback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_rect", qos_profile,
            std::bind(&RobustAruco::image_callback, this, std::placeholders::_1));

        debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco_tracker/debug", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        has_info_ = false;
        fps_start_time_ = this->get_clock()->now();

        cv::aruco::DetectorParameters detectorParams;
        detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
        detector_ = cv::aruco::ArucoDetector(dictionary, detectorParams);

        RCLCPP_INFO(this->get_logger(),
            "Robust ArUco ready. Processing every %d frames. "
            "Flip rejection: %.0f deg. Filter reset: %.1fs.",
            process_every_n_frames_,
            this->get_parameter("flip_rejection_threshold_deg").as_double(),
            filter_reset_timeout_);
    }

private:
    int frame_counter_  = 0;
    int fps_frame_count_ = 0;
    int process_every_n_frames_;
    double filter_reset_timeout_;
    double flip_threshold_rad_;
    rclcpp::Time fps_start_time_;

    std::vector<int> last_ids_;
    std::vector<std::vector<cv::Point2f>> last_corners_;

    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (has_info_) return;
        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        dist_coeffs_   = cv::Mat(1, msg->d.size(), CV_64F);
        for (int i = 0; i < 9; i++)
            camera_matrix_.at<double>(i / 3, i % 3) = msg->k[i];
        for (size_t i = 0; i < msg->d.size(); i++)
            dist_coeffs_.at<double>(0, i) = msg->d[i];
        has_info_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera info received.");
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            if (!has_info_) return;

            frame_counter_++;

            // --- 1HZ FPS LOG ---
            fps_frame_count_++;
            double elapsed = (this->get_clock()->now() - fps_start_time_).seconds();
            if (elapsed >= 1.0) {
                RCLCPP_INFO(this->get_logger(),
                    "Camera: %d fps | ArUco processing: %d fps | Markers visible: %zu",
                    fps_frame_count_,
                    fps_frame_count_ / process_every_n_frames_,
                    last_ids_.size());
                fps_frame_count_ = 0;
                fps_start_time_  = this->get_clock()->now();
            }

            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            if (!cv_ptr || cv_ptr->image.empty()) return;

            // ----------------------------------------------------------------
            // POSE ESTIMATION — only runs every N frames
            // ----------------------------------------------------------------
            bool do_process = (frame_counter_ % process_every_n_frames_ == 0);

            if (do_process) {
                std::vector<int> ids;
                std::vector<std::vector<cv::Point2f>> corners;
                detector_.detectMarkers(cv_ptr->image, corners, ids);

                last_ids_     = ids;
                last_corners_ = corners;

                for (size_t i = 0; i < ids.size(); i++) {
                    int current_id  = ids[i];
                    double current_size = (current_id == small_id_) ? small_size_ : default_size_;
                    double alpha        = (current_id == small_id_) ? alpha_small_ : alpha_default_;

                    std::vector<cv::Point3f> objPoints;
                    float s = current_size / 2.0f;
                    objPoints.push_back(cv::Point3f(-s,  s, 0));
                    objPoints.push_back(cv::Point3f( s,  s, 0));
                    objPoints.push_back(cv::Point3f( s, -s, 0));
                    objPoints.push_back(cv::Point3f(-s, -s, 0));

                    cv::Vec3d rvec_raw, tvec_raw;
                    cv::solvePnP(objPoints, corners[i], camera_matrix_, dist_coeffs_,
                                 rvec_raw, tvec_raw, false, cv::SOLVEPNP_IPPE_SQUARE);

                    // Initialise filter entry if first time seeing this marker
                    if (filters_.find(current_id) == filters_.end()) {
                        filters_[current_id] = MarkerFilter();
                        filters_[current_id].last_seen = this->get_clock()->now();
                    }

                    tf2::Quaternion q_raw = get_quat_from_rvec(rvec_raw);
                    auto& filter = filters_[current_id];

                    // --------------------------------------------------------
                    // FIX 1: FILTER RESET ON RE-DETECTION
                    // If marker was unseen for > timeout, reset filter so it
                    // snaps to the new position instead of drifting from stale state
                    // --------------------------------------------------------
                    double gap = (this->get_clock()->now() - filter.last_seen).seconds();
                    if (filter.initialized && gap > filter_reset_timeout_) {
                        RCLCPP_INFO(this->get_logger(),
                            "M%d: Lost for %.2fs — resetting filter, snapping to new pose",
                            current_id, gap);
                        filter.initialized = false;
                    }
                    filter.last_seen = this->get_clock()->now();

                    cv::Vec3d tvec_final;
                    tf2::Quaternion q_final;

                    if (!filter.initialized) {
                        // First detection or post-reset — accept directly, no filtering
                        tvec_final         = tvec_raw;
                        q_final            = q_raw;
                        filter.initialized = true;

                    } else {
                        // --------------------------------------------------------
                        // FIX 2: 180° FLIP REJECTION
                        // IPPE_SQUARE has pose ambiguity — at certain angles it
                        // returns the mirror solution causing yaw to jump 180°.
                        // Measure rotation change from last frame and reject if
                        // it exceeds the threshold.
                        // --------------------------------------------------------
                        tf2::Quaternion q_diff = filter.q.inverse() * q_raw;
                        q_diff.normalize();
                        double angle = 2.0 * std::acos(std::min(1.0, std::abs(q_diff.w())));

                        if (angle > flip_threshold_rad_) {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                "M%d: Flip rejected (%.1f deg > %.1f deg) — holding last pose",
                                current_id,
                                angle * 180.0 / M_PI,
                                flip_threshold_rad_ * 180.0 / M_PI);
                            // Republish last known good pose without updating filter
                            ensure_publisher_exists(current_id);
                            publish_marker_pose(filter.tvec, filter.q, msg->header, current_id);
                            publish_tf_quat(filter.tvec, filter.q, msg->header.stamp, current_id);
                            continue;
                        }

                        // Valid frame — apply EMA filter
                        tvec_final = alpha * tvec_raw + (1.0 - alpha) * filter.tvec;
                        q_final    = filter.q.slerp(q_raw, alpha);
                    }

                    filter.tvec = tvec_final;
                    filter.q    = q_final;

                    ensure_publisher_exists(current_id);
                    publish_marker_pose(tvec_final, q_final, msg->header, current_id);
                    publish_tf_quat(tvec_final, q_final, msg->header.stamp, current_id);
                }
            }

            // ----------------------------------------------------------------
            // DEBUG IMAGE — every N*2 frames, always shows last known detections
            // ----------------------------------------------------------------
            if (frame_counter_ % (process_every_n_frames_ * 2) == 0) {
                cv::Mat debug_img = cv_ptr->image.clone(); 
                if (!last_ids_.empty()) {
                    cv::aruco::drawDetectedMarkers(debug_img, last_corners_, last_ids_);

                    for (size_t i = 0; i < last_ids_.size(); i++) {
                        int current_id      = last_ids_[i];
                        double current_size = (current_id == small_id_) ? small_size_ : default_size_;

                        if (filters_.find(current_id) != filters_.end() &&
                            filters_[current_id].initialized)
                        {
                            cv::Mat rot_mat(3, 3, CV_64F);
                            tf2::Matrix3x3 tf_mat(filters_[current_id].q);
                            for (int r = 0; r < 3; r++)
                                for (int c = 0; c < 3; c++)
                                    rot_mat.at<double>(r, c) = tf_mat[r][c];

                            cv::Vec3d rvec_draw;
                            cv::Rodrigues(rot_mat, rvec_draw);
                            cv::drawFrameAxes(debug_img, camera_matrix_, dist_coeffs_,
                                              rvec_draw, filters_[current_id].tvec, current_size);
                        }
                    }   
                }
               debug_pub_->publish(
                *cv_bridge::CvImage(msg->header, "mono8", debug_img).toImageMsg());
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Callback Error: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Unknown Error");
        }
    }

    void ensure_publisher_exists(int id) {
        if (marker_pubs_.find(id) == marker_pubs_.end()) {
            std::string topic_name = "/aruco/marker_" + std::to_string(id);
            auto qos = rclcpp::QoS(1)
                .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                .history(rclcpp::HistoryPolicy::KeepLast);
            marker_pubs_[id] = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, qos);

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
            t.header.stamp    = stamp;
            t.header.frame_id = camera_frame_;
            t.child_frame_id  = "marker_" + std::to_string(id);
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

    void publish_marker_pose(cv::Vec3d tvec, tf2::Quaternion q,
                             std_msgs::msg::Header header, int id) {
        geometry_msgs::msg::PoseStamped p;
        p.header             = header;
        p.pose.position.x    = tvec[0];
        p.pose.position.y    = tvec[1];
        p.pose.position.z    = tvec[2];
        p.pose.orientation.x = q.x();
        p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z();
        p.pose.orientation.w = q.w();
        marker_pubs_[id]->publish(p);
    }

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr       image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr          debug_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                 tf_broadcaster_;
    std::map<int, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> marker_pubs_;

    std::map<int, MarkerFilter>  filters_;
    cv::aruco::ArucoDetector     detector_;
    cv::Mat                      camera_matrix_, dist_coeffs_;
    bool                         has_info_;
    double                       default_size_, small_size_;
    double                       alpha_small_, alpha_default_;
    int                          small_id_, target_id_;
    std::string                  camera_frame_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobustAruco>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
