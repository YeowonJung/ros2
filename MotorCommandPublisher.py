#먼저 모터 제어 노드를 실행
ros2 run your_package dc_motor_1_node
ros2 run your_package dc_motor_2_node
ros2 run your_package stepper_motor_node
ros2 run your_package actuator_node

#명령 발행 코드를 실행하여 모터에 명령을 발행
ros2 run your_package motor_command_publisher


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_publisher')

        # DC Motor 1에 대한 퍼블리셔 생성
        self.dc_motor_1_publisher = self.create_publisher(String, 'dc_motor_1_command', 10)
        # DC Motor 2에 대한 퍼블리셔 생성
        self.dc_motor_2_publisher = self.create_publisher(String, 'dc_motor_2_command', 10)
        # Stepper Motor에 대한 퍼블리셔 생성
        self.stepper_motor_publisher = self.create_publisher(String, 'stepper_motor_command', 10)
        # Linear Actuator에 대한 퍼블리셔 생성
        self.actuator_publisher = self.create_publisher(String, 'actuator_command', 10)

        # 1초 대기 후 명령을 순차적으로 전송
        time.sleep(1)
        self.run_motor_sequence()

    def run_motor_sequence(self):
        """
        모터 명령을 순차적으로 발행하는 함수입니다. 각 모터에 대해 forward와 backward 명령을 실행합니다.
        """

        # DC Motor 1: forward at speed 50 for 3 seconds
        self.send_dc_motor_1_command('forward', 50)
        time.sleep(3)
        self.send_dc_motor_1_command('backward', 50)
        time.sleep(3)

        # DC Motor 2: forward at speed 60 for 3 seconds
        self.send_dc_motor_2_command('forward', 60)
        time.sleep(3)
        self.send_dc_motor_2_command('backward', 60)
        time.sleep(3)

        # Stepper Motor: forward for 200 steps
        self.send_stepper_motor_command('forward', 200)
        time.sleep(2)
        self.send_stepper_motor_command('backward', 200)
        time.sleep(2)

        # Linear Actuator: extend for 3 seconds, then retract
        self.send_actuator_command('extend', 50)
        time.sleep(3)
        self.send_actuator_command('retract', 50)
        time.sleep(3)

    def send_dc_motor_1_command(self, direction, speed):
        """
        DC Motor 1에 명령을 발행하는 함수입니다.
        direction: 'forward' 또는 'backward'
        speed: 속도 (0-100)
        """
        msg = String()
        msg.data = f'{direction} {speed}'
        self.dc_motor_1_publisher.publish(msg)
        self.get_logger().info(f'Published to DC Motor 1: {msg.data}')

    def send_dc_motor_2_command(self, direction, speed):
        """
        DC Motor 2에 명령을 발행하는 함수입니다.
        direction: 'forward' 또는 'backward'
        speed: 속도 (0-100)
        """
        msg = String()
        msg.data = f'{direction} {speed}'
        self.dc_motor_2_publisher.publish(msg)
        self.get_logger().info(f'Published to DC Motor 2: {msg.data}')

    def send_stepper_motor_command(self, direction, steps):
        """
        Stepper Motor에 명령을 발행하는 함수입니다.
        direction: 'forward' 또는 'backward'
        steps: 회전할 스텝 수
        """
        msg = String()
        msg.data = f'{direction} {steps}'
        self.stepper_motor_publisher.publish(msg)
        self.get_logger().info(f'Published to Stepper Motor: {msg.data}')

    def send_actuator_command(self, direction, speed):
        """
        Linear Actuator에 명령을 발행하는 함수입니다.
        direction: 'extend' 또는 'retract'
        speed: 속도 (0-100)
        """
        msg = String()
        msg.data = f'{direction} {speed}'
        self.actuator_publisher.publish(msg)
        self.get_logger().info(f'Published to Actuator: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    motor_command_publisher = MotorCommandPublisher()

    try:
        # 퍼블리셔가 계속 동작하면서 명령을 발행하도록 유지
        rclpy.spin(motor_command_publisher)
    except KeyboardInterrupt:
        pass

    # 종료 시 노드를 파괴하고 ROS 종료
    motor_command_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
