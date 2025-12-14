#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from smbus2 import SMBus, i2c_msg

I2C_BUS = 14
PCA_ADDRESSES = [0x50, 0x54]

MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06

class PCA9685:
    def __init__(self, bus, addr):
        self.bus = bus
        self.addr = addr
        print(f"[PCA9685] Initializing at address 0x{addr:02X}")

        # Reset
        self.write_register(MODE1, 0x00)
        self.write_register(MODE2, 0x04)  # totem pole output

        # Set PWM frequency to 50 Hz
        self.set_pwm_freq(50)

    def write_register(self, register, value):
        self.bus.write_byte_data(self.addr, register, value)

    def set_pwm_freq(self, freq_hz):
        prescale_val = int(25000000.0 / (4096 * freq_hz) - 1)
        old_mode = self.bus.read_byte_data(self.addr, MODE1)
        new_mode = (old_mode & 0x7F) | 0x10

        self.write_register(MODE1, new_mode)
        self.write_register(PRESCALE, prescale_val)
        self.write_register(MODE1, old_mode)
        rclpy.sleep(0.005)
        self.write_register(MODE1, old_mode | 0x80)

    def set_servo_pulse(self, channel, pulse_us):
        ticks = int((pulse_us * 4096) / 20000)
        reg = LED0_ON_L + 4 * channel
        self.write_register(reg,     0)
        self.write_register(reg + 1, 0)
        self.write_register(reg + 2, ticks & 0xFF)
        self.write_register(reg + 3, ticks >> 8)


class TurretServoNode(Node):
    def __init__(self):
        super().__init__("turret_servo_node")

        self.get_logger().info("Searching for PCA9685 on I2C bus 13...")
        self.bus = SMBus(I2C_BUS)

        self.pca = self.find_pca9685()
        if self.pca is None:
            raise RuntimeError("No PCA9685 found on bus 13")

        self.subscription = self.create_subscription(
            Int32,
            "/servo_x_cmd",
            self.servo_callback,
            10
        )

    def find_pca9685(self):
        for addr in PCA_ADDRESSES:
            try:
                self.bus.read_byte_data(addr, MODE1)
                self.get_logger().info(f"PCA9685 found at 0x{addr:02X}")
                return PCA9685(self.bus, addr)
            except Exception:
                continue
        return None

    def servo_callback(self, msg):
        angle = msg.data
        pulse = 1500 + (angle * 5)
        pulse = max(500, min(2500, pulse))
        self.pca.set_servo_pulse(0, pulse)


def main(args=None):
    rclpy.init(args=args)
    node = TurretServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
