#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool

import serial
import time


class WindowSerialNode(Node):
    def __init__(self):
        super().__init__('window_serial_node')

        # 🔥 CHANGE THIS TO YOUR STABLE PATH
        self.port =  "/dev/ttyACM1"
       # self.port = "/dev/serial/by-id/usb-Arduino_LLC_Arduino_Leonardo-if00"


        self.baud = 115200
        self.ser = None
        self.connected = False
        self.reconnect_timer = None

        # ROS Subscribers
        self.best_sub = self.create_subscription(
            Int32, 'best_target_window', self.on_best_window, 10
        )
        self.fire_sub = self.create_subscription(
            Bool, 'fire_command', self.on_fire_command, 10
        )

        self.get_logger().info(
            f"WindowSerialNode starting... opening serial: {self.port} @ {self.baud}"
        )

        # Try initial connect
        self.try_open_serial()

    # -------------------------------------------------------
    # SERIAL CONNECTION MANAGEMENT
    # -------------------------------------------------------

    def try_open_serial(self):
        """Attempt to open serial port. If fails, schedule reconnect."""
        if self.connected:
            return

        try:
            self.get_logger().info(
                f"Opening serial port {self.port} at {self.baud}..."
            )
            self.ser = serial.Serial(self.port, self.baud, timeout=1)

            # Reset Leonardo cleanly
            self.ser.dtr = False
            time.sleep(0.1)
            self.ser.dtr = True

            time.sleep(2.0)  # allow Leonardo to reboot

            self.connected = True
            self.get_logger().info(
                "Serial connected — Arduino ready."
            )

            # Stop reconnect timer if running
            if self.reconnect_timer is not None:
                self.reconnect_timer.cancel()
                self.reconnect_timer = None

        except Exception as e:
            self.connected = False
            self.get_logger().warn(f"Serial open failed: {e}")

            # Start reconnect attempts
            if self.reconnect_timer is None:
                self.reconnect_timer = self.create_timer(1.0, self.on_reconnect_timer)

    def on_reconnect_timer(self):
        """Keep trying to reconnect until successful."""
        if not self.connected:
            self.try_open_serial()
        else:
            if self.reconnect_timer:
                self.reconnect_timer.cancel()
                self.reconnect_timer = None

    def write_line(self, line: str, is_fire: bool = False):
        """Safely write to serial and auto-reconnect on disconnect."""
        if not self.connected or self.ser is None:
            return

        try:
            data = (line + "\n").encode("utf-8")
            self.ser.write(data)
            self.ser.flush()

            if is_fire:
                self.get_logger().info("Sent to Arduino: FIRE")
            else:
                self.get_logger().info(f"Sent to Arduino: {line}")

        except (serial.SerialException, OSError) as e:
            # USB disconnect → Errno 5 → Arduino rebooted → reconnect
            if is_fire:
                self.get_logger().error(f"Error writing FIRE: {e}")
            else:
                self.get_logger().error(f"Error writing: {e}")

            self.connected = False

            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

            # Begin reconnect loop
            if self.reconnect_timer is None:
                self.reconnect_timer = self.create_timer(1.0, self.on_reconnect_timer)

    # -------------------------------------------------------
    # ROS CALLBACKS
    # -------------------------------------------------------

    def on_best_window(self, msg: Int32):
        self.write_line(str(msg.data))

    def on_fire_command(self, msg: Bool):
        if msg.data:
            self.write_line("FIRE", is_fire=True)

    # -------------------------------------------------------
    # CLEANUP
    # -------------------------------------------------------

    def destroy_node(self):
        if self.reconnect_timer:
            self.reconnect_timer.cancel()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WindowSerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
