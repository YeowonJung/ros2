import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # PID 제어를 위한 변수 초기화
        self.kp = 0.01  # 비례 상수
        self.angular_tolerance = 0.05  # 허용 오차 (라디안)

    def rotate_to_object(self, angle_error):
        # 회전 속도 계산
        twist_msg = Twist()
        twist_msg.angular.z = self.kp * angle_error  # 비례 제어
        if abs(angle_error) < self.angular_tolerance:
            twist_msg.angular.z = 0.0  # 오차가 작을 경우 정지

        # 모터에 회전 명령 보내기
        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass

    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
