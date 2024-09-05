pip install opencv-python
sudo apt install ros-<ros_distro>-cv-bridge ros-<ros_distro>-image-transport

#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class FireDetectionNode:
    def __init__(self):
        rospy.init_node('fire_detection_node', anonymous=True)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.coord_pub = rospy.Publisher('/fire_detection/coordinates', Point, queue_size=10)

    def image_callback(self, data):
        # Convert the ROS image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # Convert to HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the color range for detecting fire (red/yellow)
        lower_bound = np.array([0, 50, 50])
        upper_bound = np.array([10, 255, 255])

        # Create mask
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find the center of the fire-like region
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Ignore small areas (noise)
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2

                # Publish coordinates
                fire_coords = Point()
                fire_coords.x = center_x
                fire_coords.y = center_y
                fire_coords.z = 0  # Z-axis is unused in 2D detection
                self.coord_pub.publish(fire_coords)

                rospy.loginfo(f"Fire detected at: x={center_x}, y={center_y}")

if __name__ == '__main__':
    try:
        node = FireDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

