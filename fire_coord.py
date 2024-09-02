import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import torch
from models.common import DetectMultiBackend
from utils.torch_utils import select_device
from utils.general import non_max_suppression

class FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_detection_node')
        self.publisher_ = self.create_publisher(Point, 'fire_location', 10)
        self.timer = self.create_timer(0.1, self.detect_fire)  # 주기적으로 YOLO 감지 실행

        # YOLOv5 모델 로드
        self.device = select_device('')
        self.model = DetectMultiBackend('yolov5s.pt', device=self.device)
        self.model.eval()

    def detect_fire(self):
        # 카메라에서 이미지 가져오기 (예: OpenCV로 카메라 입력 받기)
        img = ...  # 실제 카메라 또는 이미지 입력 필요
        results = self.model(img)
        results = non_max_suppression(results)  # 감지된 객체 필터링

        # 감지된 객체에서 불꽃 클래스만 찾아서 좌표 추출
        for det in results:
            if det is not None:
                for *box, conf, cls in det:
                    x_center, y_center, width, height = box
                    if int(cls) == <fire_class_id>:  # 불꽃 클래스 ID 체크
                        fire_x, fire_y = x_center, y_center

                        # ROS 2 메시지로 좌표 발행
                        fire_location = Point()
                        fire_location.x = float(fire_x)
                        fire_location.y = float(fire_y)
                        fire_location.z = 0.0  # 2D 좌표이므로 z는 0
                        self.publisher_.publish(fire_location)
                        self.get_logger().info(f'Fire detected at: {fire_x}, {fire_y}')

def main(args=None):
    rclpy.init(args=args)
    fire_detection_node = FireDetectionNode()
    rclpy.spin(fire_detection_node)
    fire_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
