#!/usr/bin/env python3
"""
Simple camera viewer for the robot's vision
Shows what the robot sees and highlights detected cyan targets
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()
        
        # Movement status tracking
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.target_distance = 0.0  # Distance to target in meters
        self.robot_status = "SYSTEM READY"  # Detailed status from robot
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )
        
        # Subscribe to cmd_vel to track movement
        self.vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.vel_callback,
            10)
            
        self.status_sub = self.create_subscription(
            String,
            '/robot_status',
            self.status_callback,
            10)
        
        # Subscribe to target distance
        self.distance_sub = self.create_subscription(
            Float32,
            '/target_distance',
            self.distance_callback,
            10
        )
        
        self.get_logger().info('🎥 Camera Viewer Started!')
        self.get_logger().info('Press "q" in the image window to quit')
        self.get_logger().info('🎮 HUD Interface Active!')
    
    def vel_callback(self, msg):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    def status_callback(self, msg):
        self.robot_status = msg.data
    
    def distance_callback(self, msg):
        """Track distance to target"""
        self.target_distance = msg.data
        
    def draw_rounded_rectangle(self, image, x, y, width, height, color, thickness=-1, radius=10):
        """Draw a rectangle with rounded corners"""
        # Create mask for rounded rectangle
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        
        # Draw rounded corners using circles
        cv2.circle(mask, (x + radius, y + radius), radius, 255, -1)
        cv2.circle(mask, (x + width - radius, y + radius), radius, 255, -1)
        cv2.circle(mask, (x + radius, y + height - radius), radius, 255, -1)
        cv2.circle(mask, (x + width - radius, y + height - radius), radius, 255, -1)
        
        # Draw rectangles to connect the circles
        cv2.rectangle(mask, (x + radius, y), (x + width - radius, y + height), 255, -1)
        cv2.rectangle(mask, (x, y + radius), (x + width, y + height - radius), 255, -1)
        
        if thickness == -1:
            # Filled rectangle
            image[mask == 255] = color
        else:
            # Draw border by subtracting inner rectangle
            inner_mask = np.zeros(image.shape[:2], dtype=np.uint8)
            if x + thickness < x + width - thickness and y + thickness < y + height - thickness:
                inner_x, inner_y = x + thickness, y + thickness
                inner_width, inner_height = width - 2*thickness, height - 2*thickness
                inner_radius = max(1, radius - thickness)
                
                cv2.circle(inner_mask, (inner_x + inner_radius, inner_y + inner_radius), inner_radius, 255, -1)
                cv2.circle(inner_mask, (inner_x + inner_width - inner_radius, inner_y + inner_radius), inner_radius, 255, -1)
                cv2.circle(inner_mask, (inner_x + inner_radius, inner_y + inner_height - inner_radius), inner_radius, 255, -1)
                cv2.circle(inner_mask, (inner_x + inner_width - inner_radius, inner_y + inner_height - inner_radius), inner_radius, 255, -1)
                
                cv2.rectangle(inner_mask, (inner_x + inner_radius, inner_y), (inner_x + inner_width - inner_radius, inner_y + inner_height), 255, -1)
                cv2.rectangle(inner_mask, (inner_x, inner_y + inner_radius), (inner_x + inner_width, inner_y + inner_height - inner_radius), 255, -1)
            
            border_mask = cv2.subtract(mask, inner_mask)
            image[border_mask == 255] = color
    
    def draw_hud_panel(self, image, x, y, width, height, color=(0, 255, 255), header_text=None):
        """Draw a modern HUD panel with rounded corners and dark header"""
        # Create overlay for transparency
        overlay = image.copy()
        
        # Draw main panel with rounded corners - dark background
        temp = np.zeros_like(image)
        self.draw_rounded_rectangle(temp, x, y, width, height, (15, 15, 15), -1, radius=12)
        
        # Blend panel with transparency
        mask = (temp > 0).any(axis=2).astype(np.uint8) * 255
        alpha = 0.85
        image_with_panel = np.where(mask[:,:,None] > 0, 
                                     cv2.addWeighted(overlay, 1-alpha, temp, alpha, 0), 
                                     overlay)
        image[:] = image_with_panel
        
        if header_text:
            # Draw subtle header separator line
            cv2.line(image, (x + 8, y + 32), (x + width - 8, y + 32), (40, 40, 40), 1)
            
            # Header text - clean and thin with colored text
            cv2.putText(image, header_text, (x + 10, y + 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to HSV for cyan detection
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Define cyan color range (same as robot uses)
            lower_cyan = np.array([80, 50, 50])
            upper_cyan = np.array([100, 255, 255])
            
            # Create mask
            mask = cv2.inRange(hsv, lower_cyan, upper_cyan)
            
            # Apply morphological operations
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Draw contours and info on original image
            display_image = cv_image.copy()
            height, width = display_image.shape[:2]
            center_x, center_y = width // 2, height // 2
            
            # ===== LEFT SIDE HUD - SIMPLIFIED FOR NON-TECHNICAL USERS =====
            left_margin = 15
            current_y = 15
            
            # WHAT IS THE ROBOT DOING? PANEL
            # Determine color based on status content
            if "EMERGENCY" in self.robot_status:
                status_color = (0, 0, 255)  # Red
                status_icon = "STOP"
            elif "REACHED" in self.robot_status or "HOLDING" in self.robot_status:
                status_color = (0, 255, 0)  # Green
                status_icon = "REACHED"
            elif "BACKING" in self.robot_status:
                status_color = (255, 0, 255)  # Magenta
                status_icon = "BACKING UP"
            elif "APPROACHING" in self.robot_status or "ALIGNING" in self.robot_status:
                status_color = (0, 255, 255)  # Cyan
                status_icon = "MOVING"
            elif "SEARCHING" in self.robot_status:
                status_color = (0, 165, 255)  # Orange
                status_icon = "SEARCHING"
            elif "TRACKING" in self.robot_status:
                status_color = (0, 255, 255)  # Cyan
                status_icon = "TRACKING"
            else:
                status_color = (100, 100, 100)  # Gray
                status_icon = "IDLE"
            
            self.draw_hud_panel(display_image, left_margin, current_y, 280, 115, status_color, "WHAT IS THE ROBOT DOING?")
            
            # Simple status icon (start below header with padding)
            cv2.putText(display_image, status_icon, (left_margin + 10, current_y + 48), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
            
            # Simplified explanation
            if "EMERGENCY" in self.robot_status:
                explanation = "Too close! Stopping."
            elif "REACHED" in self.robot_status or "HOLDING" in self.robot_status:
                explanation = "At perfect distance."
            elif "BACKING" in self.robot_status:
                explanation = "Target too close."
            elif "APPROACHING" in self.robot_status:
                explanation = "Getting closer to target."
            elif "ALIGNING" in self.robot_status:
                explanation = "Turning to face target."
            elif "SEARCHING" in self.robot_status:
                explanation = "Looking for target..."
            elif "TRACKING" in self.robot_status:
                explanation = "Following target movement."
            elif "STABILIZING" in self.robot_status:
                explanation = "Holding position steady."
            else:
                explanation = "Ready to start."
            
            cv2.putText(display_image, explanation, (left_margin + 10, current_y + 72), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            
            # Speed indicator with visual bar
            speed_label = f"Speed: {abs(self.linear_vel):.2f} m/s"
            cv2.putText(display_image, speed_label, (left_margin + 10, current_y + 92), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # Visual speed bar
            bar_width = int(min(abs(self.linear_vel) * 600, 180))  # Max 0.3 m/s = 180px
            bar_color = (0, 255, 0) if self.linear_vel >= 0 else (0, 0, 255)
            cv2.rectangle(display_image, (left_margin + 10, current_y + 100), 
                         (left_margin + 10 + bar_width, current_y + 110), bar_color, -1)
            cv2.rectangle(display_image, (left_margin + 10, current_y + 100), 
                         (left_margin + 190, current_y + 110), (100, 100, 100), 1)
            
            current_y += 130
            
            # ===== TARGET DETECTION =====
            if contours:
                # Find largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 100:  # Minimum area threshold
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    
                    # Draw cooler target brackets
                    track_color = (0, 255, 255)
                    corner_len = int(min(w, h) * 0.3)
                    thickness = 2
                    
                    # Top-Left
                    cv2.line(display_image, (x, y), (x + corner_len, y), track_color, thickness)
                    cv2.line(display_image, (x, y), (x, y + corner_len), track_color, thickness)
                    # Top-Right
                    cv2.line(display_image, (x + w, y), (x + w - corner_len, y), track_color, thickness)
                    cv2.line(display_image, (x + w, y), (x + w, y + corner_len), track_color, thickness)
                    # Bottom-Left
                    cv2.line(display_image, (x, y + h), (x + corner_len, y + h), track_color, thickness)
                    cv2.line(display_image, (x, y + h), (x, y + h - corner_len), track_color, thickness)
                    # Bottom-Right
                    cv2.line(display_image, (x + w, y + h), (x + w - corner_len, y + h), track_color, thickness)
                    cv2.line(display_image, (x + w, y + h), (x + w, y + h - corner_len), track_color, thickness)
                    
                    # Draw contour outline (thin)
                    cv2.drawContours(display_image, [largest_contour], -1, (0, 255, 255), 1)
                    
                    # Calculate and draw centroid
                    M = cv2.moments(largest_contour)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        
                        # Draw crosshair at center of target
                        ch_size = 10
                        cv2.line(display_image, (cx - ch_size, cy), (cx + ch_size, cy), (0, 0, 255), 2)
                        cv2.line(display_image, (cx, cy - ch_size), (cx, cy + ch_size), (0, 0, 255), 2)
                        cv2.circle(display_image, (cx, cy), 5, (0, 0, 255), 1)
                        
                        # Calculate offset and coverage
                        offset = (cx - center_x) / (width / 2.0)
                        fov_coverage = max(w / width, h / height) * 100
                        
                        # TARGET INFO PANEL (continue on left side)
                        self.draw_hud_panel(display_image, left_margin, current_y, 280, 120, (0, 255, 255), "TARGET INFORMATION")
                        
                        # Status (start below header with padding)
                        cv2.putText(display_image, "TARGET FOUND", (left_margin + 10, current_y + 48), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                        
                        # Distance with visual indicator
                        dist_text = f"Distance: {self.target_distance:.2f}m"
                        cv2.putText(display_image, dist_text, (left_margin + 10, current_y + 68), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                        
                        # Size indicator (how much of screen it fills)
                        size_text = f"Size in view: {fov_coverage:.0f}%"
                        cv2.putText(display_image, size_text, (left_margin + 10, current_y + 88), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                        
                        # Alignment status with simple visual
                        if abs(offset) > 0.15:
                            align_color = (0, 165, 255)  # Orange
                            align_text = "Turning to center..."
                            # Draw arrow showing direction
                            arrow_x = left_margin + 190
                            arrow_y = current_y + 110
                            if offset < 0:
                                cv2.arrowedLine(display_image, (arrow_x + 10, arrow_y), (arrow_x - 10, arrow_y), 
                                              align_color, 2, tipLength=0.5)
                            else:
                                cv2.arrowedLine(display_image, (arrow_x - 10, arrow_y), (arrow_x + 10, arrow_y), 
                                              align_color, 2, tipLength=0.5)
                        else:
                            align_color = (0, 255, 0)
                            align_text = "Centered [OK]"
                            # Draw circle indicator instead of checkmark
                            cv2.circle(display_image, (left_margin + 195, current_y + 110), 8, align_color, 2)
                            cv2.circle(display_image, (left_margin + 195, current_y + 110), 4, align_color, -1)
                        
                        cv2.putText(display_image, align_text, (left_margin + 10, current_y + 113), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, align_color, 1)
            else:
                # TARGET INFO - No Target
                self.draw_hud_panel(display_image, left_margin, current_y, 280, 85, (100, 100, 100), "TARGET INFORMATION")
                cv2.putText(display_image, "NO TARGET DETECTED", (left_margin + 10, current_y + 48), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
                cv2.putText(display_image, "Robot is scanning the area", (left_margin + 10, current_y + 70), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
            
            # Show both views
            cv2.imshow('🎯 TURTLEBOT3 - VISION TRACKING SYSTEM', display_image)
            cv2.imshow('📡 TARGET DETECTION - CYAN FILTER', mask)
            
            # Wait for key press (1ms)
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.get_logger().info('🛑 Shutting down HUD...')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    viewer = CameraViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
