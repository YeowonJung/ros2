import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import lgpio

IN_A2 = 23
IN_B2 = 24
PWM2 = 25
EN_DIAG2 = 5

class DCMotor2Controller(Node):
    def __init__(self):
        super().__init__('dc_motor_2_controller')
        self.subscription = self.create_subscription(
            String, 'dc_motor_2_command', self.listener_callback, 10)
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, IN_A2, 0)
        lgpio.gpio_claim_output(self.chip, IN_B2, 0)
        lgpio.gpio_claim_output(self.chip, PWM2, 0)
        lgpio.gpio_claim_output(self.chip, EN_DIAG2, 1)

    def listener_callback(self, msg):
        command = msg.data.split()
        direction = command[0]
        speed = int(command[1])
        self.control_motor(direction, speed)

    def control_motor(self, direction, speed):
        if direction == 'forward':
            lgpio.gpio_write(self.chip, IN_A2, 1)
            lgpio.gpio_write(self.chip, IN_B2, 0)
        elif direction == 'backward':
            lgpio.gpio_write(self.chip, IN_A2, 0)
            lgpio.gpio_write(self.chip, IN_B2, 1)
        self.get_logger().info(f'Motor 2: {direction} at speed {speed}')

    def destroy_node(self):
        super().destroy_node()
        lgpio.gpiochip_close(self.chip)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = DCMotor2Controller()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
