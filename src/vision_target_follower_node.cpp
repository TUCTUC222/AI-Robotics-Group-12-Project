#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
                                  closest_distance_(10.0)
    {
        // Create publisher for velocity commands
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
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
                    // Calculate center of the detected target
                    cv::Moments moments = cv::moments(contours[max_contour_idx]);
                    if (moments.m00 > 0) {
                        int center_x = static_cast<int>(moments.m10 / moments.m00);
                        
                        // Normalize x position to [-1, 1] (left to right)
                        target_x_center_ = (center_x - image.cols / 2.0) / (image.cols / 2.0);
                        target_detected_ = true;
                        
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                            "🎯 CYAN TARGET DETECTED! Position: %.2f (area: %.0f)", 
                            target_x_center_, max_area);
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
        
        const float TARGET_DISTANCE = 0.30;  // Stop 30cm from target
        const float DANGER_DISTANCE = 0.25;  // Emergency stop distance
        
        // PRIORITY 1: Check if we've reached the target
        if (target_detected_ && closest_distance_ <= TARGET_DISTANCE) {
            // TARGET REACHED!
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "✅ TARGET REACHED! Stopped at %.2fm", closest_distance_);
        }
        // PRIORITY 2: Emergency stop if too close
        else if (closest_distance_ < DANGER_DISTANCE) {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "⚠️ TOO CLOSE! Distance: %.2fm", closest_distance_);
        }
        // PRIORITY 3: Target visible - track it!
        else if (target_detected_) {
            // Turn toward target based on its position in the image
            float turn_rate = -target_x_center_ * 1.5;  // Proportional control
            
            // Limit turn rate
            if (turn_rate > 0.8) turn_rate = 0.8;
            if (turn_rate < -0.8) turn_rate = -0.8;
            
            twist.angular.z = turn_rate;
            
            // Move forward speed based on how centered the target is
            if (std::abs(target_x_center_) < 0.15) {
                // Target is centered - move straight
                if (closest_distance_ < 0.6) {
                    twist.linear.x = 0.1;  // Slow approach
                } else {
                    twist.linear.x = 0.3;  // Normal speed
                }
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "🎯 LOCKED ON! Approaching... (%.2fm)", closest_distance_);
            } else {
                // Target off-center - turn more, move slower
                twist.linear.x = 0.1;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "🔄 Turning toward target (offset: %.2f)", target_x_center_);
            }
        }
        // PRIORITY 4: Target not visible - search
        else {
            // Rotate slowly to search for target
            twist.linear.x = 0.0;
            twist.angular.z = 0.4;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "🔍 Searching for CYAN target...");
        }
        
        publisher_->publish(twist);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    
    bool target_detected_;
    double target_x_center_;  // Normalized position: -1 (left) to +1 (right)
    float closest_distance_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionTargetFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
