from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1) Servo + FIRE Arduino serial node
        Node(
            package='servo_serial_pkg',
            executable='servo_serial_node',
            name='servo_serial_node',
            output='screen'
        ),
        # 2) Brain node (aim → move → fire)
        Node(
            package='servo_serial_pkg',
            executable='brain_node',
            name='brain_node',
            output='screen'
        ),
        # 3) VESC Twist node (drives the base)
        Node(
            package='ucsd_robocar_actuator2_pkg',
            executable='vesc_twist_node',
            name='vesc_twist_node',
            output='screen'
        ),
    ])
