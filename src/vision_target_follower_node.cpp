#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <vector>
#include <cmath>

class VisionTargetFollowerNode : public rclcpp::Node
{
public:
    VisionTargetFollowerNode() : Node("vision_target_follower_node"),
                                  target_detected_(false),
                                  target_x_center_(0.0),
                                  closest_distance_(10.0),
                                  target_width_ratio_(0.0),
                                  target_height_ratio_(0.0),
                                  target_edges_visible_(true),
                                  target_edges_clipped_(false),
                                  target_reached_(false),
                                  stable_counter_(0)
    {
        // Create publisher for velocity commands
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Create publisher for target distance
        distance_publisher_ = this->create_publisher<std_msgs::msg::Float32>("target_distance", 10);
        
        // Create publisher for robot status
        status_publisher_ = this->create_publisher<std_msgs::msg::String>("robot_status", 10);
        
        // Subscribe to camera images
        auto image_qos = rclcpp::QoS(rclcpp::KeepLast(10));
        image_qos.best_effort();
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera", image_qos,
            std::bind(&VisionTargetFollowerNode::image_callback, this, std::placeholders::_1));
        
        // Subscribe to LIDAR scan data
        auto scan_qos = rclcpp::QoS(rclcpp::KeepLast(10));
        scan_qos.best_effort();
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", scan_qos,
            std::bind(&VisionTargetFollowerNode::scan_callback, this, std::placeholders::_1));
        
        // Create timer for control loop (20Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&VisionTargetFollowerNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "🤖 Vision Target Follower Node started!");
        RCLCPP_INFO(this->get_logger(), "📷 Looking for CYAN/TEAL colored target cylinder...");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert ROS image message to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;
            
            // Convert BGR to HSV for better color detection
            cv::Mat hsv_image;
            cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
            
            // Define range for CYAN/TEAL color in HSV
            // Cyan: Hue ~85-100 (teal/cyan range)
            cv::Scalar lower_cyan(80, 50, 50);    // Lower bound
            cv::Scalar upper_cyan(100, 255, 255);  // Upper bound
            
            // Create mask for cyan pixels
            cv::Mat mask;
            cv::inRange(hsv_image, lower_cyan, upper_cyan, mask);
            
            // Apply morphological operations to reduce noise
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
            
            // Find contours in the mask
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            if (!contours.empty()) {
                // Find the largest contour (most likely the target)
                double max_area = 0;
                int max_contour_idx = -1;
                
                for (size_t i = 0; i < contours.size(); i++) {
                    double area = cv::contourArea(contours[i]);
                    if (area > max_area && area > 100) {  // Minimum area threshold
                        max_area = area;
                        max_contour_idx = i;
                    }
                }
                
                if (max_contour_idx >= 0) {
                    // Calculate bounding box and center of the detected target
                    cv::Rect bbox = cv::boundingRect(contours[max_contour_idx]);
                    cv::Moments moments = cv::moments(contours[max_contour_idx]);
                    
                    if (moments.m00 > 0) {
                        int center_x = static_cast<int>(moments.m10 / moments.m00);
                        
                        // Calculate how much of the frame the target occupies (both dimensions)
                        target_width_ratio_ = static_cast<float>(bbox.width) / image.cols;
                        target_height_ratio_ = static_cast<float>(bbox.height) / image.rows;
                        
                        // CHECK IF ALL EDGES ARE VISIBLE with safety margin
                        const int EDGE_MARGIN = 15;  // Safety margin - stop before edges clip
                        const int CLIP_MARGIN = 2;    // Actual clipping detection
                        
                        bool left_safe = bbox.x > EDGE_MARGIN;
                        bool right_safe = (bbox.x + bbox.width) < (image.cols - EDGE_MARGIN);
                        bool top_safe = bbox.y > EDGE_MARGIN;
                        bool bottom_safe = (bbox.y + bbox.height) < (image.rows - EDGE_MARGIN);
                        
                        bool left_clipped = bbox.x <= CLIP_MARGIN;
                        bool right_clipped = (bbox.x + bbox.width) >= (image.cols - CLIP_MARGIN);
                        bool top_clipped = bbox.y <= CLIP_MARGIN;
                        bool bottom_clipped = (bbox.y + bbox.height) >= (image.rows - CLIP_MARGIN);
                        
                        bool all_edges_safe = left_safe && right_safe && top_safe && bottom_safe;
                        bool any_edge_clipped = left_clipped || right_clipped || top_clipped || bottom_clipped;
                        
                        target_edges_visible_ = all_edges_safe && !any_edge_clipped;
                        target_edges_clipped_ = any_edge_clipped;
                        
                        // Find the maximum dimension ratio (limiting factor for FOV)
                        // Note: edge penalty is applied in control_loop, not here
                        float max_dimension_ratio = std::max(target_width_ratio_, target_height_ratio_);
                        
                        // Normalize x position to [-1, 1] (left to right)
                        // Scale the offset based on target size - larger targets need less aggressive turns
                        float raw_offset = (center_x - image.cols / 2.0) / (image.cols / 2.0);
                        
                        // Apply size-based damping: when target is large (close), reduce sensitivity
                        // Use the larger dimension for damping to prevent oscillation
                        float size_damping = 1.0f - std::min(max_dimension_ratio * 1.5f, 0.7f);
                        target_x_center_ = raw_offset * size_damping;
                        
                        target_detected_ = true;
                        
                        if (all_edges_safe && !any_edge_clipped) {
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                "🎯 TARGET FULLY VISIBLE: offset=%.2f W=%.0f%% H=%.0f%%", 
                                target_x_center_, target_width_ratio_ * 100, target_height_ratio_ * 100);
                        } else {
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                "⚠️ TARGET EDGES CLIPPED: offset=%.2f W=%.0f%% H=%.0f%%", 
                                target_x_center_, target_width_ratio_ * 100, target_height_ratio_ * 100);
                        }
                    }
                } else {
                    target_detected_ = false;
                }
            } else {
                target_detected_ = false;
            }
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge exception: %s", e.what());
        }
    }
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Get minimum distance in front cone (for stopping distance)
        float min_dist = scan->range_max;
        int num_readings = scan->ranges.size();
        
        for (int i = 0; i < num_readings; i++) {
            float angle = scan->angle_min + i * scan->angle_increment;
            float angle_deg = angle * 180.0 / M_PI;
            
            // Front 60-degree cone
            if (angle_deg >= -30.0 && angle_deg <= 30.0) {
                float range = scan->ranges[i];
                if (std::isfinite(range) && range > scan->range_min && range < min_dist) {
                    min_dist = range;
                }
            }
        }
        
        closest_distance_ = min_dist;
    }
    
    void control_loop()
    {
        auto twist = geometry_msgs::msg::Twist();
        std::string status_msg = "IDLE";
        
        // Control thresholds
        const float MIN_FOV_COVERAGE = 0.20f;  // Start approaching if target is smaller than this
        const float DANGER_DISTANCE = 0.30f;    // Emergency stop if closer than this
        
        // PRIORITY 1: Emergency stop if too close
        if (closest_distance_ < DANGER_DISTANCE) {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            status_msg = "EMERGENCY STOP: TOO CLOSE";
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "⚠️ TOO CLOSE! Distance: %.2fm", closest_distance_);
        }
        // PRIORITY 2: Target visible - actively track it with FOV awareness!
        else if (target_detected_) {
            // EDGE-BASED DISTANCE CONTROL - Simple rule: if edges visible, approach; if not, back up
            
            // Adaptive centering threshold
            float centering_threshold = 0.20f;
            
            // TURN CONTROL - Always turn if target is off center
            const float TURN_DEADZONE = 0.05;  // Very small deadzone for responsiveness
            if (std::abs(target_x_center_) > TURN_DEADZONE) {
                // Target off-center - turn toward it (even if "target reached")
                float turn_rate = -target_x_center_ * 1.2;  // Proportional control
                
                // Limit turn rate
                if (turn_rate > 0.8) turn_rate = 0.8;
                if (turn_rate < -0.8) turn_rate = -0.8;
                
                twist.angular.z = turn_rate;
                
                // Reset target reached if we need to realign
                if (target_reached_ && std::abs(target_x_center_) > 0.12) {
                    target_reached_ = false;
                    stable_counter_ = 0;
                }
            } else {
                // Target within deadzone - don't turn
                twist.angular.z = 0.0;
            }
            
            // DISTANCE CONTROL - Edge-based logic with safety margin
            if (std::abs(target_x_center_) < centering_threshold) {
                // Target is centered - adjust distance based on edge visibility
                
                float max_fov_coverage = std::max(target_width_ratio_, target_height_ratio_);
                
                if (target_edges_clipped_) {
                    // Edge is actually clipped - back up immediately!
                    twist.linear.x = -0.20;  // Faster backup
                    stable_counter_ = 0;
                    target_reached_ = false;
                    status_msg = "BACKING UP: EDGES CLIPPED";
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "⚠️ EDGES CLIPPED! Backing up to keep target visible!");
                } else if (max_fov_coverage < MIN_FOV_COVERAGE) {
                    // Target too small - approach quickly
                    twist.linear.x = 0.30;  // Faster approach from far away
                    stable_counter_ = 0;
                    target_reached_ = false;
                    status_msg = "APPROACHING: TARGET FAR";
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "➡️ TARGET TOO FAR! Approaching... (FOV: %.0f%%)", max_fov_coverage * 100);
                } else if (target_edges_visible_) {
                    // All edges are comfortably within safety margin (15px) - can approach
                    if (max_fov_coverage < 0.60f) {
                        twist.linear.x = 0.15;  // Medium speed careful approach
                        stable_counter_ = 0;
                        target_reached_ = false;
                        status_msg = "APPROACHING: OPTIMIZING DISTANCE";
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "➡️ Approaching slowly... FOV: %.0f%%", max_fov_coverage * 100);
                    } else {
                        // Perfect - edges safely visible and target is large
                        twist.linear.x = 0.0;
                        
                        // Only increment stability if we're also centered (not turning)
                        if (std::abs(target_x_center_) < TURN_DEADZONE) {
                            if (!target_reached_) {
                                stable_counter_++;
                                status_msg = "STABILIZING: HOLDING POSITION";
                                
                                // If stable for 2 seconds (40 iterations at 20Hz), declare target reached
                                if (stable_counter_ >= 40) {
                                    target_reached_ = true;
                                    status_msg = "TARGET REACHED: HOLDING";
                                    RCLCPP_INFO(this->get_logger(), 
                                        "🎯✅ TARGET REACHED! Stable at %.0f%% FOV, %.2fm", 
                                        max_fov_coverage * 100, closest_distance_);
                                } else {
                                    // Still stabilizing - holding position
                                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                        "🎯 PERFECT FOV! Holding at %.0f%% (stabilizing: %d/40)", 
                                        max_fov_coverage * 100, stable_counter_);
                                }
                            } else {
                                // Already reached - stay stopped
                                status_msg = "TARGET REACHED: HOLDING";
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "🎯✅ TARGET REACHED! Holding position.");
                            }
                        } else {
                            // Turning to track - reset stability
                            stable_counter_ = 0;
                            target_reached_ = false;
                            status_msg = "TRACKING: FOLLOWING TARGET";
                        }
                    }
                } else {
                    // Edges are in danger zone (between 2-15px from border) - STOP and hold position!
                    twist.linear.x = 0.0;
                    stable_counter_ = 0;
                    target_reached_ = false;
                    status_msg = "HOLDING: EDGE SAFETY MARGIN";
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "⚠️ Near edge! Holding at %.0f%% FOV", max_fov_coverage * 100);
                }
            } else {
                // Target off-center - turn while approaching
                twist.linear.x = 0.15;  // Faster approach while turning
                stable_counter_ = 0;  // Reset stability counter
                target_reached_ = false;  // Reset target reached flag
                status_msg = "ALIGNING: TURNING TO TARGET";
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "🔄 Turning toward target (offset: %.2f)", target_x_center_);
            }
        }
        // PRIORITY 4: Target not visible - search
        else {
            // Rotate slowly to search for target
            twist.linear.x = 0.0;
            twist.angular.z = 0.4;
            stable_counter_ = 0;  // Reset stability counter
            target_reached_ = false;  // Reset target reached flag
            status_msg = "SEARCHING: SCANNING AREA";
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "🔍 Searching for CYAN target...");
        }
        
        publisher_->publish(twist);
        
        // Publish distance to target for HUD display
        auto distance_msg = std_msgs::msg::Float32();
        distance_msg.data = closest_distance_;
        distance_publisher_->publish(distance_msg);
        
        // Publish robot status
        auto status_msg_ros = std_msgs::msg::String();
        status_msg_ros.data = status_msg;
        status_publisher_->publish(status_msg_ros);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    
    bool target_detected_;
    double target_x_center_;  // Normalized position: -1 (left) to +1 (right)
    float closest_distance_;
    float target_width_ratio_;   // Target width as fraction of frame width
    float target_height_ratio_;  // Target height as fraction of frame height
    bool target_edges_visible_;  // Whether all edges are within safety margin (15px)
    bool target_edges_clipped_;  // Whether any edges are actually clipped (within 2px)
    bool target_reached_;        // Whether the robot has reached the target
    int stable_counter_;         // Count how many iterations the robot has been stable
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionTargetFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
