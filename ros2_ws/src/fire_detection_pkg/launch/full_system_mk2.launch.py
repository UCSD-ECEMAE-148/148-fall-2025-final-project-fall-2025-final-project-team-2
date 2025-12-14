
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1) Fire detection + summit (OAK camera)
        Node(
            package='fire_detection_pkg',
            executable='fire_detection_node',
            name='fire_detection_node',
            output='screen'
        ),

        # 2) Arduino serial node (servo + FIRE)
        Node(
            package='servo_serial_pkg',
            executable='servo_serial_node',
            name='servo_serial_node',
            output='screen'
        ),

        # 3) Brain node (sequence: servo -> VESC -> FIRE)
        Node(
            package='servo_serial_pkg',
            executable='brain_node',
            name='brain_node',
            output='screen'
        ),

        Node(
            package='ucsd_robocar_actuator2_pkg',
            executable='vesc_twist_node',
            name='vesc_twist_node',
            output='screen'
        ),
    ])
