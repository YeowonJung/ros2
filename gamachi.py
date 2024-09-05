import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_detection_node')
        
        # 퍼블리셔 생성
        self.publisher_ = self.create_publisher(Point, 'fire_detection/coordinates', 10)

    def publish_coordinates(self, x_center, y_center):
        # 좌표 퍼블리시
        point_msg = Point()
        point_msg.x = x_center
        point_msg.y = y_center
        point_msg.z = 0.0  # Z축은 2D 감지에서 사용하지 않음

        self.publisher_.publish(point_msg)
        self.get_logger().info(f'불꽃 좌표 퍼블리시됨: x={x_center}, y={y_center}')


def main(args=None):
    rclpy.init(args=args)

    fire_detection_node = FireDetectionNode()

    try:
        rclpy.spin(fire_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        fire_detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
