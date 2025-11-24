#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <vector>
#include <cmath>

// Bug2 algorithm states
enum class Bug2State {
    GO_TO_GOAL,        // Direct approach to target
    ROTATE_TO_WALL,    // Rotate 90° to face wall
    WALL_FOLLOW        // Follow wall until can leave
};

// Simple 2D position struct
struct Position {
    double x;
    double y;
    
    Position() : x(0.0), y(0.0) {}
    Position(double x_, double y_) : x(x_), y(y_) {}
    
    double distance_to(const Position& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

class VisionTargetFollowerNode : public rclcpp::Node
{
public:
    VisionTargetFollowerNode() : Node("vision_target_follower_node"),
                                  target_detected_(false),
                                  target_x_center_(0.0),
                                  closest_distance_(10.0),
                                  left_distance_(10.0),
                                  right_distance_(10.0),
                                  target_width_ratio_(0.0),
                                  target_height_ratio_(0.0),
                                  target_edges_visible_(true),
                                  target_edges_clipped_(false),
                                  target_reached_(false),
                                  stable_counter_(0),
                                  last_target_x_(0.0),
                                  last_target_dist_(0.0),
                                  bug2_state_(Bug2State::GO_TO_GOAL),
                                  wall_follow_left_(true),
                                  target_heading_(0.0),
                                  robot_theta_(0.0),
                                  wall_detected_visually_(false),
                                  rotation_complete_(false)
    {
        // Initialize last detection time
        last_detection_time_ = this->get_clock()->now();

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
        
        // Subscribe to odometry for position tracking (Bug2 algorithm)
        auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10));
        odom_qos.best_effort();
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", odom_qos,
            std::bind(&VisionTargetFollowerNode::odom_callback, this, std::placeholders::_1));
        
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
                        float raw_offset = (center_x - image.cols / 2.0) / (image.cols / 2.0);
                        
                        // Apply minimal size-based damping only for very large/close targets to prevent oscillation
                        // Reduced damping to maintain better centering response
                        float size_damping = 1.0f - std::min(max_dimension_ratio * 0.5f, 0.3f);
                        target_x_center_ = raw_offset * size_damping;
                        
                        target_detected_ = true;
                        
                        // Update last known position info
                        last_detection_time_ = this->get_clock()->now();
                        last_target_x_ = target_x_center_;
                        last_target_dist_ = closest_distance_;
                        
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
        // Reset distances
        float min_front = scan->range_max;
        float min_left = scan->range_max;
        float min_right = scan->range_max;
        
        int num_readings = scan->ranges.size();
        
        for (int i = 0; i < num_readings; i++) {
            float angle = scan->angle_min + i * scan->angle_increment;
            float angle_deg = angle * 180.0 / M_PI;
            float range = scan->ranges[i];
            
            if (std::isfinite(range) && range > scan->range_min) {
                // Front cone (-30 to 30)
                if (angle_deg >= -30.0 && angle_deg <= 30.0) {
                    if (range < min_front) min_front = range;
                }
                // Left cone (30 to 75) - widened for better avoidance
                else if (angle_deg > 30.0 && angle_deg <= 75.0) {
                    if (range < min_left) min_left = range;
                }
                // Right cone (-75 to -30) - widened for better avoidance
                else if (angle_deg >= -75.0 && angle_deg < -30.0) {
                    if (range < min_right) min_right = range;
                }
            }
        }
        
        closest_distance_ = min_front;
        left_distance_ = min_left;
        right_distance_ = min_right;
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // DEBUG: Print odom reception (throttle to avoid spam)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "📍 Odom received: x=%.2f, y=%.2f", msg->pose.pose.position.x, msg->pose.pose.position.y);

        robot_position_.x = msg->pose.pose.position.x;
        robot_position_.y = msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        double qw = msg->pose.pose.orientation.w;
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        
        // Convert quaternion to yaw (Z-axis rotation)
        robot_theta_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }
    
    // Helper: Normalize angle to [-pi, pi]
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    // Helper: Calculate angle from robot to target (based on last known position)
    double angle_to_target() {
        if (!target_detected_) return 0.0;
        
        // Target is in front of us, offset by target_x_center_
        // This is a rough estimate - in reality we'd track target position in world frame
        return target_x_center_ * 0.5;  // Simple proportional estimate
    }
    
    // Helper: Check if we're on the M-line (line from start to goal)
    bool is_on_m_line() {
        if (start_position_.distance_to(robot_position_) < 0.1) return true;
        
        // Calculate distance from robot to M-line
        double line_length = start_position_.distance_to(hit_point_);
        if (line_length < 0.01) return false;
        
        // Use cross product to find perpendicular distance
        double dx = hit_point_.x - start_position_.x;
        double dy = hit_point_.y - start_position_.y;
        double px = robot_position_.x - start_position_.x;
        double py = robot_position_.y - start_position_.y;
        
        double cross = std::abs(dx * py - dy * px);
        double distance_to_line = cross / line_length;
        
        return distance_to_line < 0.15;  // Within 15cm of M-line
    }
    
    // Helper: Check if we should leave the wall
    bool should_leave_wall() {
        if (!is_on_m_line()) return false;
        
        // Only leave if we're closer to goal than when we hit the wall
        double current_distance = robot_position_.distance_to(hit_point_);
        double hit_distance = start_position_.distance_to(hit_point_);
        
        return current_distance < (hit_distance - 0.2);  // 20cm closer
    }
    
    void control_loop()
    {
        auto twist = geometry_msgs::msg::Twist();
        std::string status_msg = "IDLE";
        
        // Constants
        const float DANGER_DIST = 0.5f;
        const float OBSTACLE_DIST = 0.8f;
        const float WALL_FOLLOW_DIST = 0.4f;  // Desired distance from wall
        const float TURN_DEADZONE = 0.03f;
        
        // BUG2 STATE MACHINE
        switch (bug2_state_) {
            
        case Bug2State::GO_TO_GOAL:
        {
            // NORMAL VISION-BASED TRACKING - Use existing logic
            const float MIN_FOV_COVERAGE = 0.20f;
            
            if (target_detected_) {
                // Check for obstacles blocking path to target
                bool obstacle_blocking = (closest_distance_ < OBSTACLE_DIST);
                
                if (obstacle_blocking && closest_distance_ < DANGER_DIST) {
                    // HIT AN OBSTACLE - Switch to Bug2 wall-following
                    RCLCPP_WARN(this->get_logger(), "🚧 OBSTACLE BLOCKING TARGET! Starting Bug2 wall-following...");
                    
                    // Record hit point and start position for M-line calculation
                    hit_point_ = robot_position_;
                    if (start_position_.x == 0.0 && start_position_.y == 0.0) {
                        start_position_ = robot_position_;
                    }
                    
                    // Decide which side to follow: choose side with more space
                    // If left > right, turn left (+90), so wall is on RIGHT.
                    // If right > left, turn right (-90), so wall is on LEFT.
                    bool turn_left = (left_distance_ > right_distance_);
                    wall_follow_left_ = !turn_left;
                    
                    RCLCPP_INFO(this->get_logger(), "📍 Hit point: (%.2f, %.2f) - Turning %s, Following %s wall", 
                        hit_point_.x, hit_point_.y, turn_left ? "LEFT" : "RIGHT", wall_follow_left_ ? "LEFT" : "RIGHT");
                    
                    // Transition to rotation state
                    bug2_state_ = Bug2State::ROTATE_TO_WALL;
                    rotation_complete_ = false;
                    
                    // If following LEFT wall, we turned RIGHT (-90)
                    // If following RIGHT wall, we turned LEFT (+90)
                    // MODIFIED: Turn only 60 degrees to keep wall in FOV (Vision-Assisted)
                    double turn_angle = wall_follow_left_ ? -M_PI/3.0 : M_PI/3.0;
                    target_heading_ = robot_theta_ + turn_angle;
                    target_heading_ = normalize_angle(target_heading_);
                    
                    twist.linear.x = 0.0;
                    twist.angular.z = 0.0;
                    status_msg = "BUG2: HIT OBSTACLE - TURNING";
                } else {
                    // NO OBSTACLE - Use normal vision tracking
                    float centering_threshold = 0.20f;
                    bool obstacle_front = (closest_distance_ < OBSTACLE_DIST);
                    
                    // TURN CONTROL - Simple vision-based tracking
                    if (std::abs(target_x_center_) > TURN_DEADZONE) {
                        twist.angular.z = -target_x_center_ * 2.5;  // Increased gain for faster centering
                        
                        // Limit turn rate
                        if (twist.angular.z > 1.0) twist.angular.z = 1.0;
                        if (twist.angular.z < -1.0) twist.angular.z = -1.0;
                    } else {
                        twist.angular.z = 0.0;
                    }
                    
                    // Reset target reached if we are turning significantly
                    if (std::abs(twist.angular.z) > 0.2) {
                        target_reached_ = false;
                        stable_counter_ = 0;
                    }
                    
                    // DISTANCE CONTROL - Edge-based logic with safety margin
                    if (std::abs(target_x_center_) < centering_threshold) {
                        // Target is centered - adjust distance based on edge visibility
                        
                        float max_fov_coverage = std::max(target_width_ratio_, target_height_ratio_);
                        
                        // Check for obstacles FIRST - safety priority!
                        if (obstacle_front && closest_distance_ < DANGER_DIST) {
                            // STOP! Obstacle too close
                            twist.linear.x = 0.0;
                            twist.angular.z = 0.0;
                            stable_counter_ = 0;
                            target_reached_ = false;
                            status_msg = "STOPPED: OBSTACLE TOO CLOSE";
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                "🛑 OBSTACLE TOO CLOSE (%.2fm)! STOPPED!", closest_distance_);
                } else if (target_edges_clipped_) {
                    // Edge is actually clipped - back up immediately!
                    twist.linear.x = -0.20;  // Faster backup
                    stable_counter_ = 0;
                    target_reached_ = false;
                    status_msg = "BACKING UP: EDGES CLIPPED";
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "⚠️ EDGES CLIPPED! Backing up to keep target visible!");
                } else if (obstacle_front) {
                    // Obstacle ahead but not critical - slow down significantly
                    twist.linear.x = 0.05;
                    stable_counter_ = 0;
                    target_reached_ = false;
                    status_msg = "OBSTACLE AHEAD: SLOWING DOWN";
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "⚠️ Obstacle at %.2fm - slowing down", closest_distance_);
                } else if (max_fov_coverage < MIN_FOV_COVERAGE) {
                    // Target too small - approach quickly (no obstacles)
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
                twist.linear.x = 0.15;  // Approach while turning
                
                stable_counter_ = 0;  // Reset stability counter
                target_reached_ = false;  // Reset target reached flag
                status_msg = "ALIGNING: TURNING TO TARGET";
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "🔄 Turning toward target (offset: %.2f)", target_x_center_);
            }
        }
    }
        // PRIORITY 4: Target not visible - search strategy
        else {
            // Calculate time since last detection
            auto current_time = this->get_clock()->now();
            auto time_since_lost = (current_time - last_detection_time_).seconds();
            
            // PHASE 1: Turn towards last known position (0-2 seconds after loss)
            // Only if target was significantly off-center when lost
            if (time_since_lost < 2.0 && std::abs(last_target_x_) > 0.2) {
                // Turn in the direction it was last seen
                // last_target_x_ > 0 means it was on the right -> turn right (negative z)
                float turn_dir = (last_target_x_ > 0) ? -0.6 : 0.6;
                twist.angular.z = turn_dir;
                
                // Only move forward if path is clear
                if (closest_distance_ > 0.6f) {
                    twist.linear.x = 0.05; // Move slightly forward too
                } else {
                    twist.linear.x = 0.0;  // Just turn in place if obstacle ahead
                }
                status_msg = "SEARCH: TURNING TO LAST POS";
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "🔍 Lost target on %s! Turning...", (last_target_x_ > 0 ? "RIGHT" : "LEFT"));
            }
            // PHASE 2: Move to last known position (2-X seconds after loss)
            // Move forward to check where it was (WITH obstacle avoidance!)
            else {
                // Calculate how long to drive based on last known distance
                // Speed = 0.2 m/s
                // Duration = distance / speed
                float travel_duration = last_target_dist_ / 0.2f;
                
                // Cap duration to reasonable limits (min 2s, max 15s)
                if (travel_duration < 2.0f) travel_duration = 2.0f;
                if (travel_duration > 15.0f) travel_duration = 15.0f;
                
                float phase2_end_time = 2.0f + travel_duration;
                
                if (time_since_lost < phase2_end_time) {
                    // Check if path is clear before moving forward
                    const float SEARCH_MIN_DISTANCE = 0.6f;
                    
                    if (closest_distance_ > SEARCH_MIN_DISTANCE) {
                        // Path clear - move forward to last known position
                        twist.linear.x = 0.2;
                        twist.angular.z = 0.0;
                        status_msg = "SEARCH: MOVING TO LAST POS";
                        
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "🔍 Checking last known location (%.1fm away)...", last_target_dist_);
                    } else {
                        // Obstacle ahead - can't reach last position
                        // Switch to Bug2 wall-following to go around it
                        RCLCPP_WARN(this->get_logger(), "🚧 OBSTACLE BLOCKING SEARCH! Starting Bug2 wall-following...");
                        
                        // Record hit point
                        hit_point_ = robot_position_;
                        if (start_position_.x == 0.0 && start_position_.y == 0.0) {
                            start_position_ = robot_position_;
                        }
                        
                        // Decide which side to follow
                        bool turn_left = (left_distance_ > right_distance_);
                        wall_follow_left_ = !turn_left;
                        
                        RCLCPP_INFO(this->get_logger(), "📍 Search blocked at (%.2f, %.2f) - Turning %s", 
                            hit_point_.x, hit_point_.y, turn_left ? "LEFT" : "RIGHT");
                        
                        // Transition to rotation state
                        bug2_state_ = Bug2State::ROTATE_TO_WALL;
                        rotation_complete_ = false;
                        
                        // Turn 60 degrees to keep wall in FOV
                        double turn_angle = wall_follow_left_ ? -M_PI/3.0 : M_PI/3.0;
                        target_heading_ = robot_theta_ + turn_angle;
                        target_heading_ = normalize_angle(target_heading_);
                        
                        twist.linear.x = 0.0;
                        twist.angular.z = 0.0;
                        status_msg = "BUG2: SEARCH BLOCKED - TURNING";
                    }
                }
                // PHASE 3: Scan area (after reaching last pos)
                else {
                    // Standard rotation search
                    twist.linear.x = 0.0;
                    twist.angular.z = 0.4;
                    status_msg = "SEARCH: SCANNING AREA";
                    
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "🔍 Target lost. Scanning area...");
                }
            }
            
            stable_counter_ = 0;  // Reset stability counter
            target_reached_ = false;  // Reset target reached flag
        }
        break;
    }
        
        case Bug2State::ROTATE_TO_WALL:
        {
            // Rotate until we face the wall (target_heading_)
            double error = normalize_angle(target_heading_ - robot_theta_);
            
            if (std::abs(error) > 0.1) {
                twist.angular.z = (error > 0) ? 0.5 : -0.5;
                twist.linear.x = 0.0;
                status_msg = "BUG2: ROTATING TO WALL";
            } else {
                // Rotation complete
                twist.angular.z = 0.0;
                bug2_state_ = Bug2State::WALL_FOLLOW;
                status_msg = "BUG2: STARTING WALL FOLLOW";
                RCLCPP_INFO(this->get_logger(), "🔄 Rotation complete! Starting wall follow...");
            }
            break;
        }
        
        case Bug2State::WALL_FOLLOW:
        {
            // WALL FOLLOWING LOGIC
            float side_distance = wall_follow_left_ ? left_distance_ : right_distance_;
            float error = side_distance - WALL_FOLLOW_DIST;
            
            // P-controller for wall following
            float p_gain = 2.0;
            float turn_cmd = 0.0;
            float bias = 0.3; // Bias to keep camera angled towards wall
            
            if (wall_follow_left_) {
                // Wall on left.
                // Bias: Turn LEFT (+) to look at wall
                // Error: If too close (-), Turn RIGHT (-)
                turn_cmd = p_gain * error + bias; 
            } else {
                // Wall on right.
                // Bias: Turn RIGHT (-) to look at wall
                // Error: If too close (-), Turn LEFT (+)
                turn_cmd = -p_gain * error - bias;
            }
            
            // Corner handling / Front obstacle avoidance
            if (closest_distance_ < WALL_FOLLOW_DIST + 0.1) {
                // Wall/Obstacle in front! Turn away!
                // If following left wall, turn right (-).
                // If following right wall, turn left (+).
                turn_cmd = wall_follow_left_ ? -0.8 : 0.8;
                twist.linear.x = 0.0; // Stop to turn
                status_msg = "BUG2: CORNER DETECTED";
            } else {
                // Path clear, move forward
                twist.linear.x = 0.15;
                status_msg = "BUG2: FOLLOWING WALL";
            }
            
            twist.angular.z = turn_cmd;
            
            // Clamp turn rate
            if (twist.angular.z > 1.0) twist.angular.z = 1.0;
            if (twist.angular.z < -1.0) twist.angular.z = -1.0;
            
            // EXIT CONDITION: Target visible and path clear
            if (target_detected_) {
                // Check if the path to the target is actually clear
                // We can see it, but is there a wall in front?
                if (closest_distance_ > OBSTACLE_DIST) {
                    RCLCPP_INFO(this->get_logger(), "🎯 Target spotted! Leaving wall...");
                    bug2_state_ = Bug2State::GO_TO_GOAL;
                    status_msg = "BUG2: TARGET FOUND, LEAVING WALL";
                }
            }
            
            break;
        }
            
        } // End switch
        
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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    
    bool target_detected_;
    double target_x_center_;  // Normalized position: -1 (left) to +1 (right)
    float closest_distance_;
    float left_distance_;
    float right_distance_;
    float target_width_ratio_;   // Target width as fraction of frame width
    float target_height_ratio_;  // Target height as fraction of frame height
    bool target_edges_visible_;  // Whether all edges are within safety margin (15px)
    bool target_edges_clipped_;  // Whether any edges are actually clipped (within 2px)
    bool target_reached_;        // Whether the robot has reached the target
    int stable_counter_;         // Count how many iterations the robot has been stable
    
    // Search state variables
    rclcpp::Time last_detection_time_;
    double last_target_x_;
    float last_target_dist_;
    
    // Bug2 algorithm variables
    Bug2State bug2_state_;
    Position robot_position_;
    Position start_position_;
    Position hit_point_;
    bool wall_follow_left_;       // True = follow wall on left, False = follow wall on right
    double target_heading_;       // Desired heading when rotating to goal
    double robot_theta_;          // Current robot orientation (yaw)
    bool wall_detected_visually_; // Whether camera sees a wall edge
    bool rotation_complete_;      // Whether current rotation is done
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionTargetFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
