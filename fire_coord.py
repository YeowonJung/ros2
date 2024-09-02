import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import torch
import cv2

import sys
import os

# YOLOv5의 디렉토리를 sys.path에 추가
sys.path.append('/path_to_yolov5_directory')  # YOLOv5 코드가 위치한 절대 경로로 수정하세요

from models.common import DetectMultiBackend
from utils.torch_utils import select_device
from utils.general import non_max_suppression

class FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_detection_node')
        self.publisher_ = self.create_publisher(Point, 'fire_location', 10)
        self.timer = self.create_timer(1.0, self.detect_fire)

        self.device = select_device('cpu')  # Raspberry Pi 4에서는 'cpu'로 설정
        self.model = DetectMultiBackend('best.pt', device=self.device)
        self.model.eval()

    def detect_fire(self):
        try:
            img = self.get_image()
            results = self.model(img)
            results = non_max_suppression(results)

            for det in results:
                if det is not None:
                    for *box, conf, cls in det:
                        x_center, y_center, width, height = box
                        if int(cls) == 1:  # 특정 클래스에 대한 조건을 확인하세요
                            fire_x, fire_y = x_center, y_center
                            fire_location = Point()
                            fire_location.x = float(fire_x)
                            fire_location.y = float(fire_y)
                            fire_location.z = 0.0
                            self.publisher_.publish(fire_location)
                            self.get_logger().info(f'Fire detected at: {fire_x}, {fire_y}')
        except Exception as e:
            self.get_logger().error(f'Error in detect_fire: {str(e)}')

    def get_image(self):
        # 간단한 테스트를 위한 이미지 캡처
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        cap.release()
        if not ret:
            raise RuntimeError('Failed to capture image from camera')
        return frame

def main(args=None):
    rclpy.init(args=args)
    fire_detection_node = FireDetectionNode()
    rclpy.spin(fire_detection_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
