from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1) Fire detection + best_target_window
        Node(
            package='fire_detection_pkg',
            executable='fire_detection_node',
            name='fire_detection_node',
            output='screen',
        ),

        # 2) Servo + FIRE → Arduino
        Node(
            package='servo_serial_pkg',
            executable='servo_serial_node',
            name='servo_serial_node',
            output='screen',
        ),

        # 3) Brain state machine (aim → move base → fire → cycle_complete)
        Node(
            package='servo_serial_pkg',
            executable='brain_node',
            name='brain_node',
            output='screen',
        ),

        # 4) VESC / base controller: subscribes to /cmd_vel
        # 🔴 IMPORTANT: change `package` and maybe `executable`
        # to match what you run manually when you test the car.
        #
        # Example: if you currently do:
        #   ros2 run ucsd_robocar_vesc vesc_twist_node
        # then use package='ucsd_robocar_vesc', executable='vesc_twist_node'
        Node(
            package='ucsd_robocar_vesc',      # ← FIX THIS if different
            executable='vesc_twist_node',     # ← FIX THIS if different
            name='vesc_twist_node',
            output='screen',
            parameters=[{
                # use the same values you saw printed when running it manually
                'max_rpm': 7640.0,
                'steering_polarity': 1,
                'throttle_polarity': 1,
                'max_right_steering': 0.8,
                'straight_steering': 0.5,
                'max_left_steering': 0.1,
                'steering_offset': 0.25,
                # and whatever parameter the node uses for the VESC serial port,
                # often 'port' or 'device'
                # 'port': '/dev/ttyACM1',
            }],
        ),
    ])
