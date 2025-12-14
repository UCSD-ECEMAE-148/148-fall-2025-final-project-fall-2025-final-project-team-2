from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():

    # 1️⃣ Start the servo serial node FIRST
    servo_node = Node(
        package='servo_serial_pkg',
        executable='servo_serial_node',
        name='servo_serial_node',
        output='screen'
    )

    # 2️⃣ Start the brain node AFTER servo node is running
    brain_node = Node(
        package='servo_serial_pkg',
        executable='brain_node',
        name='brain_node',
        output='screen'
    )

    brain_after_servo = RegisterEventHandler(
        OnProcessStart(
            target_action=servo_node,
            on_start=[brain_node]
        )
    )

    # 3️⃣ Start fire detection AFTER brain node is running
    fire_node = Node(
        package='fire_detection_pkg',
        executable='fire_detection_node',
        name='fire_detection_node',
        output='screen'
    )

    fire_after_brain = RegisterEventHandler(
        OnProcessStart(
            target_action=brain_node,
            on_start=[fire_node]
        )
    )

    # 4️⃣ VESC node (optional) – launched automatically after servo, brain, fire
    vesc = Node(
        package='ucsd_robocar_actuator2_pkg',
        executable='vesc_twist_node',
        name='vesc_twist_node',
        output='screen'
    )

    return LaunchDescription([
        servo_node,
        brain_after_servo,
        fire_after_brain,
        vesc,
    ])
