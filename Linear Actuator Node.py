import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import lgpio

ACTUATOR_IN_A = 20
ACTUATOR_IN_B = 21
ACTUATOR_PWM = 16

class ActuatorController(Node):
    def __init__(self):
        super().__init__('actuator_controller')
        self.subscription = self.create_subscription(
            String, 'actuator_command', self.listener_callback, 10)
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, ACTUATOR_IN_A, 0)
        lgpio.gpio_claim_output(self.chip, ACTUATOR_IN_B, 0)
        lgpio.gpio_claim_output(self.chip, ACTUATOR_PWM, 0)

    def listener_callback(self, msg):
        command = msg.data.split()
        direction = command[0]
        speed = int(command[1])
        self.control_actuator(direction, speed)

    def control_actuator(self, direction, speed):
        if direction == 'extend':
            lgpio.gpio_write(self.chip, ACTUATOR_IN_A, 1)
            lgpio.gpio_write(self.chip, ACTUATOR_IN_B, 0)
        elif direction == 'retract':
            lgpio.gpio_write(self.chip, ACTUATOR_IN_A, 0)
            lgpio.gpio_write(self.chip, ACTUATOR_IN_B, 1)
        self.get_logger().info(f'Linear Actuator: {direction} at speed {speed}')

    def destroy_node(self):
        super().destroy_node()
        lgpio.gpiochip_close(self.chip)

def main(args=None):
    rclpy.init(args=args)
    actuator_controller = ActuatorController()
    rclpy.spin(actuator_controller)
    actuator_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
