import rclpy
from rclpy.node import Node
from example_interfaces.srv import GetCoordinates
from geometry_msgs.msg import Twist

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        
        # 서비스 클라이언트 생성
        self.client = self.create_client(GetCoordinates, 'get_fire_coordinates')
        
        # 서비스가 준비될 때까지 대기
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for YOLO detection service...')
        
        # 로봇의 속도 명령 발행을 위한 퍼블리셔
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 주기적으로 화재 탐지를 시도
        self.timer = self.create_timer(0.5, self.search_for_fire)

    def search_for_fire(self):
        # 로봇을 회전시켜 화재를 탐지
        twist = Twist()
        twist.angular.z = 0.3  # 느리게 회전
        self.cmd_pub.publish(twist)
        self.send_request()

    def send_request(self):
        request = GetCoordinates.Request()
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        response = future.result()
        if response.x != -1.0:  # 화재가 감지되었을 때
            self.get_logger().info(f'Fire detected at X: {response.x}')
            self.align_to_fire(response.x)
        else:
            self.get_logger().info('No fire detected.')

    def align_to_fire(self, fire_x):
        # 카메라 중심을 기준으로 화재 중심으로 맞추기 위한 작은 회전
        error = fire_x - 0.5  # 0.5는 카메라의 중심
        twist = Twist()
        if abs(error) > 0.02:  # 허용 오차 (2%)
            twist.angular.z = -0.5 * error  # 오류 방향에 따라 회전
            self.cmd_pub.publish(twist)
            self.get_logger().info(f'Aligning X... Error: {error}')
        else:
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info('Fire X coordinate aligned with camera center. Robot ready to move.')

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
