#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist


class BrainNode(Node):
    """
    Sequence (one cycle at a time):

      1) FireDetectionNode publishes /best_target_window (0..8).
      2) servo_serial_node immediately sends that index to Arduino (servo moves).
      3) Brain waits SERVO_SETTLE_SEC.
      4) Brain publishes /cmd_vel to move the base (vesc_twist_node).
      5) Brain waits VESC_MOVE_SEC.
      6) Brain publishes /cmd_vel stop + /fire_command = True.
      7) Marks sequence as done -> next /best_target_window can start a new cycle.
    """

    SERVO_SETTLE_SEC = 0.7   # time for servo to finish turning
    VESC_MOVE_SEC = 1.0      # time for base/VESC to move

    def __init__(self):
        super().__init__('brain_node')

        self.current_target_idx = None

        # Are we currently in the middle of a sequence?
        self.sequence_active = False

        # Publish directly to existing VESC twist topic
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Publish fire trigger (servo_serial_node -> Arduino)
        self.fire_pub = self.create_publisher(Bool, 'fire_command', 10)

        # Subscribe to best target window from fire_detection_node
        self.best_sub = self.create_subscription(
            Int32,
            'best_target_window',
            self.best_target_callback,
            10
        )

        # Timers for sequencing
        self.servo_timer = None
        self.vesc_timer = None

        self.get_logger().info("BrainNode started: listening to /best_target_window")

    # -------------------- Callbacks --------------------

    def best_target_callback(self, msg: Int32):
        """
        Called whenever FireDetectionNode picks a best window.
        If no sequence is running, start a new sequence.
        If a sequence is already running, ignore this message.
        """
        idx = int(msg.data)
        self.get_logger().info(f"New best_target_window = {idx}")

        if self.sequence_active:
            # Prevent timers from constantly restarting
            self.get_logger().info("Sequence already active; ignoring new best_target_window.")
            return

        self.sequence_active = True
        self.current_target_idx = idx

        # Start servo settle timer
        self.servo_timer = self.create_timer(
            self.SERVO_SETTLE_SEC,
            self._on_servo_settled
        )
        self.get_logger().info("Started servo settle timer.")

    def _on_servo_settled(self):
        """Called after we assume the servo has finished turning."""
        # One-shot timer -> cancel it
        if self.servo_timer is not None:
            self.servo_timer.cancel()
            self.servo_timer = None

        if self.current_target_idx is None:
            self.get_logger().warn("Servo settled but no current target index.")
            self.sequence_active = False
            return

        # Convert 0..8 window index into row 0..2
        row = int(self.current_target_idx) // 3
        self.get_logger().info(f"Row for base movement = {row}")

        # Map row -> linear.x speed (tune these for your robot!)
        if row == 0:
            speed = 0.5    # forward
        elif row == 1:
            speed = 0.0    # stay
        elif row == 2:
            speed = -0.5   # backward
        else:
            self.get_logger().warn(f"Unknown row {row}, stopping.")
            speed = 0.0

        twist = Twist()
        twist.linear.x = float(speed)
        twist.angular.z = 0.0

        self.cmd_pub.publish(twist)
        self.get_logger().info(f"Published /cmd_vel linear.x = {speed}")

        # Start timer for VESC/base movement
        self.vesc_timer = self.create_timer(
            self.VESC_MOVE_SEC,
            self._on_vesc_moved
        )
        self.get_logger().info("Started VESC move timer.")

    def _on_vesc_moved(self):
        """Called after we assume the base has repositioned under the target."""
        # One-shot timer -> cancel
        if self.vesc_timer is not None:
            self.vesc_timer.cancel()
            self.vesc_timer = None

        # 1) Stop the base
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_pub.publish(stop_twist)
        self.get_logger().info("Published /cmd_vel stop (0.0)")

        # 2) Fire once
        fire_msg = Bool()
        fire_msg.data = True
        self.fire_pub.publish(fire_msg)
        self.get_logger().info("Published fire_command = True")

        # 3) Mark sequence as done so we can react to the next best_target_window
        self.sequence_active = False
        self.get_logger().info("Sequence complete; ready for next target.")

    # -------------------- Helpers --------------------

    def destroy_node(self):
        # Cleanly cancel timers on shutdown
        if self.servo_timer is not None:
            self.servo_timer.cancel()
        if self.vesc_timer is not None:
            self.vesc_timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

