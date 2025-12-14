#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist


class BrainNode(Node):
    """
    State machine controller for ONE target/cycle at a time.

    Flow for each cycle:

      1) FireDetectionNode publishes /best_target_window (0..8).
      2) BrainNode (this) receives it while in WAIT_FOR_TARGET.
      3) BrainNode publishes the same index to the servo topic (Arduino turns turret).
      4) Wait SERVO_SETTLE_SEC.
      5) Move base (via /cmd_vel) depending on the row (0, 1, 2).
      6) Wait BASE_MOVE_SEC.
      7) Stop base + publish /fire_command = True (Arduino fires cannon).
      8) Publish /cycle_complete = True so FireDetectionNode can run again.
      9) Return to WAIT_FOR_TARGET for the next window.
    """

    # Timings (tune as needed)
    SERVO_SETTLE_SEC = 0.7   # time for servo to reach target
    BASE_MOVE_SEC = .5      # time to move robot under that window
    FIRE_DELAY_SEC = 1.75

    # State constants
    STATE_WAIT_FOR_TARGET = 0
    STATE_MOVE_SERVO = 1
    STATE_WAIT_SERVO = 2
    STATE_MOVE_BASE = 3
    STATE_WAIT_BASE = 4
    STATE_FIRE = 5
    STATE_COMPLETE = 6

    def __init__(self):
        super().__init__('brain_node')
     
        self.current_row = 1 
        # Current state + target
        self.state = self.STATE_WAIT_FOR_TARGET
        self.current_target = None  # window index 0..8

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.fire_pub = self.create_publisher(Bool, 'fire_command', 10)
        # This is what your servo serial node should subscribe to
        # (change topic name here if your servo node uses a different one)
        self.servo_pub = self.create_publisher(Int32, 'servo_serial_node', 10)
        # Notifies FireDetectionNode that a full cycle is finished
        self.cycle_pub = self.create_publisher(Bool, 'cycle_complete', 10)
        self.fire_timer = None

        # Subscriber: best target from FireDetectionNode
        self.best_sub = self.create_subscription(
            Int32,
            'best_target_window',
            self.on_new_target,
            10
        )

        # Timers for servo/base waits
        self.servo_timer = None
        self.base_timer = None

        self.get_logger().info("BrainNode started. Waiting for /best_target_window...")

    # ------------------------------------------------------
    # STEP 1 — Receive new target (only when idle)
    # ------------------------------------------------------
    def on_new_target(self, msg: Int32):
        if self.state != self.STATE_WAIT_FOR_TARGET:
            # We’re in the middle of a cycle; ignore new targets
            self.get_logger().debug(
                f"Ignoring target {msg.data} because state={self.state}"
            )
            return

        self.current_target = int(msg.data)
        self.get_logger().info(f"Received NEW TARGET window {self.current_target}")

        # Move to next state and start servo step
        self.state = self.STATE_MOVE_SERVO
        self.move_servo()

    # ------------------------------------------------------
    # STEP 2 — Command servo to face correct column
    # ------------------------------------------------------
    def move_servo(self):
        if self.current_target is None:
            self.get_logger().warn("move_servo called with no current_target.")
            self.state = self.STATE_WAIT_FOR_TARGET
            return

        col = self.current_target % 3  # 0 = left, 1 = center, 2 = right
        self.get_logger().info(f"Commanding servo for column {col} (index {self.current_target})")

        # Publish the window index so the servo node forwards it to Arduino
        self.servo_pub.publish(Int32(data=self.current_target))

        # Start servo settle timer
        self.state = self.STATE_WAIT_SERVO
        self.servo_timer = self.create_timer(
            self.SERVO_SETTLE_SEC,
            self.after_servo_settled
        )

    def after_servo_settled(self):
        # One‑shot timer → cancel and clear
        if self.servo_timer is not None:
            self.servo_timer.cancel()
            self.servo_timer = None

        self.get_logger().info("Servo assumed settled.")
        self.state = self.STATE_MOVE_BASE
        self.move_base()

    # ------------------------------------------------------
    # STEP 3 — Move base based on row (0/1/2)
    # ------------------------------------------------------
    def compute_row(self, index: int) -> int:
        return index // 3     

    def move_base(self):

        if self.current_target is None:
            self.get_logger().warn("move_base called with no current_target.")
            self.state = self.STATE_WAIT_FOR_TARGET
            return

        # Compute rows
        target_row = self.compute_row(self.current_target)
        current_row = self.compute_row(self.last_window if hasattr(self, "last_window") else 4)

        row_diff = target_row - current_row
        self.get_logger().info(f"Current row = {current_row}, Target row = {target_row}, Δrow = {row_diff}")

    # Determine movement direction
        if row_diff > 0:
            speed = -0.25     # Move DOWN (toward bottom row)
        elif row_diff < 0:
            speed = 0.25      # Move UP (toward top row)
        else:
            speed = 0.0      # Same row → no base movement

        # Duration scales with how many rows to move
        duration = abs(row_diff) * self.BASE_MOVE_SEC

        self.get_logger().info(f"Driving with speed {speed} for {duration} seconds")

        twist = Twist()
        twist.linear.x = float(speed)
        twist.angular.z = -0.25
        self.cmd_pub.publish(twist)

        # Start timer
        self.state = self.STATE_WAIT_BASE
        self.base_timer = self.create_timer(
            duration,
            self.after_base_moved
        ) 


    def after_base_moved(self):
        # One‑shot timer → cancel and clear
        if self.base_timer is not None:
            self.base_timer.cancel()
            self.base_timer = None

        # Stop base
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.cmd_pub.publish(stop)
        self.get_logger().info("Base stopped (cmd_vel = 0).")
       
        self.state = self.STATE_WAIT_BASE  # or create a STATE_WAIT_BEFORE_FIRE if you like
        self.fire_timer = self.create_timer(
            self.FIRE_DELAY_SEC,
            self._on_fire_delay_done
        )
    def _on_fire_delay_done(self):
        if self.fire_timer is not None:
            self.fire_timer.cancel()
            self.fire_timer = None

        self.state = self.STATE_FIRE
        self.fire_cannon()

    # ------------------------------------------------------
    # STEP 4 — Fire!
    # ------------------------------------------------------
    def fire_cannon(self):
        fire_msg = Bool()
        fire_msg.data = True
        self.fire_pub.publish(fire_msg)

        self.get_logger().info("🔥 fire_command = True published (telling Arduino to fire).")

        self.state = self.STATE_COMPLETE
        self.complete_cycle()

    # ------------------------------------------------------
    # STEP 5 — Notify FireDetection + reset state
    # ------------------------------------------------------
    def complete_cycle(self):
        # Tell FireDetectionNode it can run another detection
       
        done_msg = Bool()
        done_msg.data = True
        self.cycle_pub.publish(done_msg)
        self.get_logger().info("cycle_complete = True published (unlocking FireDetection).")
        self.last_window = self.current_target

        # Reset for next target
        self.last_window = self.current_target  # <--- NEW
        self.current_target = None
        self.state = self.STATE_WAIT_FOR_TARGET

        self.get_logger().info("Cycle complete. Back to WAIT_FOR_TARGET.")

    # ------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------
    def destroy_node(self):
        if self.servo_timer is not None:
            self.servo_timer.cancel()
        if self.base_timer is not None:
            self.base_timer.cancel()
            super().destroy_node()
        if self.fire_timer is not None:
            self.fire_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
