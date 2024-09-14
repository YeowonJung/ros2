import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from some_custom_msgs.msg import ObjectPosition  # 객체 좌표가 포함된 커스텀 메시지
import cv2
import math

class ObjectAlignment(Node):

    def __init__(self):
        super().__init__('object_alignment')
        self.camera_width = 640  # 카메라 해상도 가로 픽셀 수
        self.fov = math.radians(60)  # 카메라의 수평 시야각 (라디안으로 변환)
        self.center_x = self.camera_width / 2  # 화면 중앙 좌표
        
        # 객체 좌표를 구독
        self.create_subscription(ObjectPosition, '/object_position', self.object_callback, 10)
        
        # 모터 제어를 위한 publisher 설정
        self.motor_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def object_callback(self, msg):
        object_x = msg.x  # 객체의 x 좌표 (픽셀)
        angle_to_object = self.calculate_angle_to_object(object_x)
        
        # 객체와 정렬할 때까지 회전
        self.rotate_robot(angle_to_object)

    def calculate_angle_to_object(self, object_x):
        # 객체가 카메라의 중앙에서 얼마나 벗어났는지 계산
        offset = object_x - self.center_x
        
        # 오프셋을 각도로 변환
        angle = offset * (self.fov / self.camera_width)
        return angle

    def rotate_robot(self, angle):
        twist_msg = Twist()
        twist_msg.angular.z = angle * 0.01  # 간단한 비례 제어
        self.motor_pub.publish(twist_msg)

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
