#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineDetectionAndFollowing(Node):
    def __init__(self):
        super().__init__('Lane_follower')
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.process_data, 10)
        self.bridge = CvBridge()  # converting ROS images to OpenCV data
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        self.velocity = Twist()
        self.error = 0
        self.action = ""

    def send_cmd_vel(self):
        self.velocity.linear.x = 0.5
        if self.error > 0:
            self.velocity.angular.z = -0.15  # Go left (negative angular velocity)
            self.action = "Go Left"
        else:
            self.velocity.angular.z = 0.15  # Go right (positive angular velocity)
            self.action = "Go Right"

        self.publisher.publish(self.velocity)

    def process_data(self, data):
        image = self.bridge.imgmsg_to_cv2(data)  # Convert ROS image to OpenCV format

        # Define the color range for line extraction
        light_line = np.array([100, 100, 100])
        dark_line = np.array([200, 200, 200])
        mask = cv2.inRange(image, light_line, dark_line)

        # Crop the image to focus on the region of interest
        r1, c1 = 150, 0
        img = mask[r1:r1+240, c1:c1+640]

        # Find line midpoint using contour detection
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            contour = max(contours, key=cv2.contourArea)
            moments = cv2.moments(contour)
            if moments['m00'] != 0:
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])
                mid_point = cx + c1
                img[cy, cx] = 255
            else:
                mid_point = 0
        else:
            mid_point = 0

        # Calculate the error and adjust the car's behavior
        frame_mid = img.shape[1] / 2  # Use image width instead of a hardcoded value
        self.error = frame_mid - mid_point

        # Visualize the output
        cv2.putText(img, self.action, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.imshow('output image', img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = LineDetectionAndFollowing()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
