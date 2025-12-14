#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
import serial
import serial.serialutil


class WindowSerialNode(Node):
    def __init__(self):
        super().__init__('window_serial_node')

        # ---- SERIAL SETUP ----
        # Change port/baudrate if your Arduino uses something different.
        port = '/dev/ttyACM1'   # common for Arduino over USB
        baud = 115200

        try:
            self.get_logger().info(f"Opening serial port {port} at {baud} baud...")
            self.ser = serial.Serial(port, baudrate=baud, timeout=1.0)

            # --- FORCE ARDUINO RESET HERE ---
            # Toggle DTR to reset the Arduino and let it run setup()
            self.get_logger().info("Toggling DTR to reset Arduino...")
            self.ser.setDTR(False)
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            self.ser.setDTR(True)

            # Give Arduino time to reboot and initialize servo
            time.sleep(2.0)
            self.get_logger().info("Serial port open, Arduino should be initialized now.")

        except serial.serialutil.SerialException as e:
            self.get_logger().error(f"Could not open serial port {port}: {e}")
            self.ser = None

        # ---- ROS SUBSCRIBERS ----
        # We listen for the best window id and fire command.
        self.sub = self.create_subscription(
            Int32,
            'best_target_window',      # make sure this matches your topic name
            self.best_window_callback,
            10
        )

        self.fire_sub = self.create_subscription(
            Bool,
            'fire_command',
            self.fire_callback,
            10
        )

        self.get_logger().info("WindowSerialNode started, waiting for best_window_id...")

    # ------------------ Callbacks ------------------ #

    def best_window_callback(self, msg: Int32):
        window_id = msg.data
        self.get_logger().info(f"Received best_window_id = {window_id}")

        if self.ser is None:
            self.get_logger().warn("Serial port not open, cannot send to Arduino.")
            return

        command = f"{window_id}\n"
        try:
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent to Arduino: {command.strip()}")
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f"Error writing to serial: {e}")

    def fire_callback(self, msg: Bool):
        """Called when someone publishes to /fire_command."""
        if not msg.data:
            # Only fire when data == True
            self.get_logger().info("fire_command received but data is False, ignoring.")
            return

        if self.ser is None:
            self.get_logger().warn("Serial port not open, cannot send FIRE.")
            return

        command = "FIRE\n"
        try:
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info("Sent to Arduino: FIRE")
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f"Error writing FIRE to serial: {e}")

    # ------------------ Shutdown ------------------ #

    def destroy_node(self):
        # Close serial on shutdown
        if getattr(self, 'ser', None) is not None and self.ser.is_open:
            self.get_logger().info("Closing serial port")
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WindowSerialNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
