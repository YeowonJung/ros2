import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class FireSubscriber(Node):
    def __init__(self):
        super().__init__('fire_subscriber')
        self.subscription = self.create_subscription(
            Point,
            'fire_location',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'Fire detected at: x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    fire_subscriber = FireSubscriber()
    rclpy.spin(fire_subscriber)
    fire_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
