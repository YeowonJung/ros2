import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import lgpio

# DC 모터 1 핀 설정
IN_A1 = 17  # 모터 1 방향 제어 핀 A
IN_B1 = 27  # 모터 1 방향 제어 핀 B
PWM1 = 18   # 모터 1 속도 제어 핀
EN_DIAG1 = 22  # 모터 1 활성화 핀

class DCMotor1Controller(Node):
    def __init__(self):
        super().__init__('dc_motor_1_controller')
        # 'dc_motor_1_command'라는 토픽을 구독하여 모터 제어 명령을 수신합니다.
        self.subscription = self.create_subscription(
            String, 'dc_motor_1_command', self.listener_callback, 10)
        
        # GPIO 칩을 열고 모터 1의 핀들을 설정합니다.
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, IN_A1, 0)
        lgpio.gpio_claim_output(self.chip, IN_B1, 0)
        lgpio.gpio_claim_output(self.chip, PWM1, 0)
        lgpio.gpio_claim_output(self.chip, EN_DIAG1, 1)

    def listener_callback(self, msg):
        # 구독한 메시지에서 방향과 속도를 파싱합니다.
        command = msg.data.split()
        direction = command[0]
        speed = int(command[1])
        self.control_motor(direction, speed)

    def control_motor(self, direction, speed):
        # 'forward' 혹은 'backward' 명령에 따라 모터 방향을 설정합니다.
        if direction == 'forward':
            lgpio.gpio_write(self.chip, IN_A1, 1)
            lgpio.gpio_write(self.chip, IN_B1, 0)
        elif direction == 'backward':
            lgpio.gpio_write(self.chip, IN_A1, 0)
            lgpio.gpio_write(self.chip, IN_B1, 1)
        # 모터의 방향과 속도를 로그로 출력합니다.
        self.get_logger().info(f'Motor 1: {direction} at speed {speed}')

    def destroy_node(self):
        # 노드 종료 시 GPIO 칩을 닫습니다.
        super().destroy_node()
        lgpio.gpiochip_close(self.chip)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = DCMotor1Controller()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
