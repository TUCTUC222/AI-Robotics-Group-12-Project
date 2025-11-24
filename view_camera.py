#!/usr/bin/env python3
"""
Simple camera viewer for the robot's vision
Shows what the robot sees and highlights detected cyan targets
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )
        
        self.get_logger().info('🎥 Camera Viewer Started!')
        self.get_logger().info('Press "q" in the image window to quit')
        
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
            
            # Draw center crosshair
            height, width = display_image.shape[:2]
            center_x, center_y = width // 2, height // 2
            cv2.line(display_image, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2)
            cv2.line(display_image, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2)
            cv2.putText(display_image, "CENTER", (center_x + 25, center_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Draw FOV cone indicators at edges
            cv2.putText(display_image, "60° FOV", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            if contours:
                # Find largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 100:  # Minimum area threshold
                    # Draw contour
                    cv2.drawContours(display_image, [largest_contour], -1, (0, 255, 255), 3)
                    
                    # Calculate and draw centroid
                    M = cv2.moments(largest_contour)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        
                        # Draw target center
                        cv2.circle(display_image, (cx, cy), 10, (0, 0, 255), -1)
                        
                        # Draw line from center to target
                        cv2.line(display_image, (center_x, center_y), (cx, cy), (0, 255, 255), 2)
                        
                        # Calculate offset
                        offset = (cx - center_x) / (width / 2.0)
                        
                        # Display target info
                        cv2.putText(display_image, f"TARGET DETECTED", (10, 60), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        cv2.putText(display_image, f"Area: {int(area)}", (10, 90), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        cv2.putText(display_image, f"Offset: {offset:.2f}", (10, 120), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        
                        # Draw direction arrow
                        if abs(offset) > 0.15:
                            arrow_color = (0, 165, 255)  # Orange
                            direction = "LEFT" if offset < 0 else "RIGHT"
                            cv2.putText(display_image, f"TURN {direction}", (10, 150), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, arrow_color, 2)
                        else:
                            cv2.putText(display_image, "CENTERED", (10, 150), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(display_image, "NO TARGET", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Show both views
            cv2.imshow('Robot Camera View', display_image)
            cv2.imshow('Cyan Detection Mask', mask)
            
            # Wait for key press (1ms)
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.get_logger().info('Quitting...')
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
