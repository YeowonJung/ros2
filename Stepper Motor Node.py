import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import lgpio
import time

STEP_PIN = 19
DIR_PIN = 26
ENA_PIN = 13

class StepperMotorController(Node):
    def __init__(self):
        super().__init__('stepper_motor_controller')
        self.subscription = self.create_subscription(
            String, 'stepper_motor_command', self.listener_callback, 10)
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, STEP_PIN, 0)
        lgpio.gpio_claim_output(self.chip, DIR_PIN, 0)
        lgpio.gpio_claim_output(self.chip, ENA_PIN, 0)

    def listener_callback(self, msg):
        command = msg.data.split()
        direction = command[0]
        steps = int(command[1])
        self.control_motor(direction, steps)

    def control_motor(self, direction, steps):
        if direction == 'forward':
            lgpio.gpio_write(self.chip, DIR_PIN, 1)
        elif direction == 'backward':
            lgpio.gpio_write(self.chip, DIR_PIN, 0)

        lgpio.gpio_write(self.chip, ENA_PIN, 1)

        for _ in range(steps):
            lgpio.gpio_write(self.chip, STEP_PIN, 1)
            time.sleep(0.001)
            lgpio.gpio_write(self.chip, STEP_PIN, 0)
            time.sleep(0.001)

        lgpio.gpio_write(self.chip, ENA_PIN, 0)
        self.get_logger().info(f'Stepper Motor: {direction} for {steps} steps')

    def destroy_node(self):
        super().destroy_node()
        lgpio.gpiochip_close(self.chip)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = StepperMotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
