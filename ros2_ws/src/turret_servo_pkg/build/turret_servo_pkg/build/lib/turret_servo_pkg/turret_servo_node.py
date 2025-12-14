#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import board
import busio
from adafruit_pca9685 import PCA9685


class TurretServoNode(Node):
    def __init__(self):
        super().__init__("turret_servo_node")

        self.get_logger().info("Initializing PCA9685 Turret Servo Node...")

        # ----------------------------
        # I2C + PCA9685 Initialization
        # ----------------------------
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c)
            self.pca.frequency = 50  # standard servo frequency
            self.servo_channel = self.pca.channels[0]  # X-axis servo on channel 0

            self.get_logger().info("PCA9685 successfully initialized on channel 0")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize PCA9685: {e}")
            raise e

        # ----------------------------
        # Subscriber to best target window
        # ----------------------------
        self.subscription = self.create_subscription(
            Int32,
            "best_target_window",
            self.target_callback,
            10
        )

        self.get_logger().info("Turret Servo Node READY")

    # ------------------------------------------------------
    # Convert window index → servo pulse width (microseconds)
    # ------------------------------------------------------
    def window_to_pulse_us(self, window_id: int):
        """
        For now we map the 3 window indexes directly:
            window 0 -> left
            window 1 -> center
            window 2 -> right

        YOU WILL TUNE THESE VALUES LATER
        """
        if window_id == 0:
            return 1200  # left position
        elif window_id == 1:
            return 1500  # center
        elif window_id == 2:
            return 1800  # right
        else:
            return 1500  # default center

    # ------------------------------------------------------
    # Convert microsecond pulse → PCA9685 16-bit PWM
    # ------------------------------------------------------
    def pulse_us_to_pca9685(self, pulse_us: int):
        # PCA9685 pulse resolution
        pwm_frequency = 50
        period_us = 1_000_000 / pwm_frequency  # 20,000 µs for 50 Hz

        duty_cycle = int((pulse_us / period_us) * 65535)
        duty_cycle = max(0, min(65535, duty_cycle))  # clamp

        return duty_cycle

    # ------------------------------------------------------
    # Callback: move servo when we receive best window ID
    # ------------------------------------------------------
    def target_callback(self, msg: Int32):
        window_id = msg.data
        self.get_logger().info(f"Received best target window: {window_id}")

        pulse_us = self.window_to_pulse_us(window_id)
        duty_cycle = self.pulse_us_to_pca9685(pulse_us)

        self.get_logger().info(f"Moving servo → {pulse_us} us (duty {duty_cycle})")

        try:
            self.servo_channel.duty_cycle = duty_cycle
        except Exception as e:
            self.get_logger().error(f"Servo write failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TurretServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
