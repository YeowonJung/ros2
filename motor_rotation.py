import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String

class ObjectAlignment(Node):

    def __init__(self):
        super().__init__('object_alignment')

        # 카메라 해상도 (가로 픽셀 수)
        self.camera_width = 640  # 카메라 해상도 (가로)
        self.center_x = self.camera_width / 2  # 카메라의 중앙선 좌표

        # DC 모터 제어를 위한 Publisher 생성 (모터 1과 모터 2에 각각 명령 발행)
        self.left_motor_pub = self.create_publisher(String, 'dc_motor_1_command', 10)
        self.right_motor_pub = self.create_publisher(String, 'dc_motor_2_command', 10)

        # fire_location 토픽으로부터 좌표를 구독하는 Subscriber 생성
        self.create_subscription(Point, 'fire_location', self.object_callback, 10)

        # 허용 오차 (화면 중앙에서 얼마나 벗어나면 조정할지 설정)
        self.alignment_tolerance = 10  # 픽셀 단위 허용 오차

    def object_callback(self, msg):
        # fire_location 토픽에서 받은 객체의 x 좌표
        object_x = msg.x  # 객체의 x 좌표
        
        # 객체와 카메라 중앙선이 일치하도록 로봇을 회전
        self.align_robot(object_x)

    def align_robot(self, object_x):
        # 객체의 x 좌표가 카메라의 중심에서 얼마나 벗어났는지 계산
        offset = object_x - self.center_x

        # 회전 제어를 위한 명령 초기화
        left_motor_command = String()
        right_motor_command = String()

        if abs(offset) > self.alignment_tolerance:
            if offset > 0:
                # 객체가 오른쪽에 있을 경우 시계 방향으로 회전 (왼쪽 모터 느리게, 오른쪽 모터 빠르게)
                left_motor_command.data = "forward 50"  # 속도 50으로 전진
                right_motor_command.data = "backward 50"  # 속도 50으로 후진
            else:
                # 객체가 왼쪽에 있을 경우 반시계 방향으로 회전 (오른쪽 모터 느리게, 왼쪽 모터 빠르게)
                left_motor_command.data = "backward 50"
                right_motor_command.data = "forward 50"
        else:
            # 객체가 중앙에 위치하면 모터 정지
            left_motor_command.data = "forward 0"
            right_motor_command.data = "forward 0"

        # 각 모터 속도 명령을 발행
        self.left_motor_pub.publish(left_motor_command)
        self.right_motor_pub.publish(right_motor_command)

def main(args=None):
    rclpy.init(args=args)
    object_alignment = ObjectAlignment()

    try:
        rclpy.spin(object_alignment)
    except KeyboardInterrupt:
        pass

    object_alignment.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
