import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_detection_node')

        # CvBridge initialization
        self.bridge = CvBridge()

        # Subscriber to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        # Publisher for fire detection coordinates
        self.publisher_ = self.create_publisher(Point, '/fire_detection/coordinates', 10)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Convert to HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define color range for fire detection (red/orange range)
        lower_bound = np.array([0, 50, 50])
        upper_bound = np.array([10, 255, 255])

        # Create mask
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Loop through contours to find the fire-like region
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Ignore small areas
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2

                # Publish the center coordinates of the fire
                fire_coords = Point()
                fire_coords.x = float(center_x)
                fire_coords.y = float(center_y)
                fire_coords.z = 0.0  # Z-axis not used here

                self.publisher_.publish(fire_coords)

                self.get_logger().info(f"Fire detected at: x={center_x}, y={center_y}")

def main(args=None):
    rclpy.init(args=args)
    node = FireDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


