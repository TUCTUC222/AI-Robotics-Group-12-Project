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
        self.is_moving = False
        self.target_distance = 0.0  # Distance to target in meters
        self.robot_status = "SYSTEM READY"  # Detailed status from robot
        
        # HUD effects
        self.frame_count = 0
        
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
        self.is_moving = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01

    def status_callback(self, msg):
        self.robot_status = msg.data
    
    def distance_callback(self, msg):
        """Track distance to target"""
        self.target_distance = msg.data
        
    def draw_hud_panel(self, image, x, y, width, height, color=(0, 255, 255), header_text=None):
        """Draw a modern HUD panel with header"""
        # Background
        overlay = image.copy()
        cv2.rectangle(overlay, (x, y), (x + width, y + height), (20, 20, 20), -1)
        
        # Header strip
        if header_text:
            cv2.rectangle(overlay, (x, y), (x + width, y + 20), color, -1)
            
        # Darker background for better text contrast
        cv2.addWeighted(overlay, 0.8, image, 0.2, 0, image)
        
        # Border lines
        cv2.rectangle(image, (x, y), (x + width, y + height), color, 1)
        
        if header_text:
            # Header text in black
            cv2.putText(image, header_text, (x + 5, y + 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
    
    def image_callback(self, msg):
        try:
            self.frame_count += 1
            
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
            
            # ===== DRAW HUD FRAME =====
            # Modern corner brackets
            accent_len = 30
            gap = 10
            color = (0, 255, 255)
            thickness = 2
            
            # Top Left
            cv2.line(display_image, (gap, gap), (gap + accent_len, gap), color, thickness)
            cv2.line(display_image, (gap, gap), (gap, gap + accent_len), color, thickness)
            
            # Top Right
            cv2.line(display_image, (width - gap, gap), (width - gap - accent_len, gap), color, thickness)
            cv2.line(display_image, (width - gap, gap), (width - gap, gap + accent_len), color, thickness)
            
            # Bottom Left
            cv2.line(display_image, (gap, height - gap), (gap + accent_len, height - gap), color, thickness)
            cv2.line(display_image, (gap, height - gap), (gap, height - gap - accent_len), color, thickness)
            
            # Bottom Right
            cv2.line(display_image, (width - gap, height - gap), (width - gap - accent_len, height - gap), color, thickness)
            cv2.line(display_image, (width - gap, height - gap), (width - gap, height - gap - accent_len), color, thickness)
            
            # ===== LEFT SIDE HUD - ALL INFO CONSOLIDATED =====
            left_margin = 15
            current_y = 15
            
            # SYSTEM INFO PANEL
            self.draw_hud_panel(display_image, left_margin, current_y, 250, 80, (0, 255, 255), "SYSTEM INFO")
            cv2.putText(display_image, "FOV: 60 DEG", (left_margin + 10, current_y + 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            cv2.putText(display_image, f"RES: {width}x{height}", (left_margin + 10, current_y + 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            current_y += 95
            
            # ROBOT STATUS PANEL
            # Determine color based on status content
            if "EMERGENCY" in self.robot_status:
                status_color = (0, 0, 255)  # Red
            elif "STABILIZING" in self.robot_status or "HOLDING" in self.robot_status:
                status_color = (0, 255, 0)  # Green
            elif "SEARCHING" in self.robot_status:
                status_color = (0, 165, 255) # Orange
            else:
                status_color = (0, 255, 255)  # Cyan default
            
            self.draw_hud_panel(display_image, left_margin, current_y, 250, 100, status_color, "ROBOT STATUS")
            
            # Status text - Split if too long
            status_str = self.robot_status
            if ":" in status_str:
                parts = status_str.split(":")
                main_status = parts[0].strip()
                sub_status = parts[1].strip()
                
                cv2.putText(display_image, main_status, (left_margin + 10, current_y + 35), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, status_color, 1)
                cv2.putText(display_image, sub_status, (left_margin + 10, current_y + 55), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                vel_y = current_y + 70
            else:
                cv2.putText(display_image, status_str, (left_margin + 10, current_y + 40), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, status_color, 1)
                vel_y = current_y + 60
            
            # Speed display
            linear_text = f"LIN: {abs(self.linear_vel):.2f} m/s"
            cv2.putText(display_image, linear_text, (left_margin + 10, vel_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            angular_text = f"ANG: {abs(self.angular_vel):.2f} rad/s"
            cv2.putText(display_image, angular_text, (left_margin + 10, vel_y + 18), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            current_y += 115
            
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
                        
                        # TARGET DATA PANEL (continue on left side)
                        self.draw_hud_panel(display_image, left_margin, current_y, 250, 110, (0, 255, 255), "TARGET DATA")
                        
                        cv2.putText(display_image, "LOCKED", (left_margin + 10, current_y + 35), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                        cv2.putText(display_image, f"DIST: {self.target_distance:.2f}m", (left_margin + 10, current_y + 55), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                        cv2.putText(display_image, f"SIZE: {fov_coverage:.0f}%", (left_margin + 10, current_y + 73), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                        
                        # Alignment indicator
                        if abs(offset) > 0.15:
                            align_color = (0, 165, 255)  # Orange
                            direction = "LEFT" if offset < 0 else "RIGHT"
                            align_text = f"TURN {direction}"
                        else:
                            align_color = (0, 255, 0)
                            align_text = "ALIGNED"
                        cv2.putText(display_image, align_text, (left_margin + 10, current_y + 93), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, align_color, 1)
            else:
                # TARGET DATA - No Target
                self.draw_hud_panel(display_image, left_margin, current_y, 250, 70, (0, 0, 255), "TARGET DATA")
                cv2.putText(display_image, "NO SIGNAL", (left_margin + 10, current_y + 40), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                cv2.putText(display_image, "Scanning...", (left_margin + 10, current_y + 58), 
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
