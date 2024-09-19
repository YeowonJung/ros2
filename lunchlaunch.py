import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # fire_bot_pkg의 노드들
        Node(
            package='fire_bot_pkg',
            executable='motor_command_publisher',
            name='motor_command_publisher',
            output='screen'
        ),
        Node(
            package='fire_bot_pkg',
            executable='fire_detection_node',
            name='fire_detection_node',
            output='screen'
        ),
        Node(
            package='fire_bot_pkg',
            executable='fire_subscriber',
            name='fire_subscriber',
            output='screen'
        ),
        Node(
            package='fire_bot_pkg',
            executable='motor_controller',
            name='motor_controller',
            output='screen'
        ),
        Node(
            package='fire_bot_pkg',
            executable='dc_motor_1_controller',
            name='dc_motor_1_controller',
            output='screen'
        ),
        Node(
            package='fire_bot_pkg',
            executable='dc_motor_2_controller',
            name='dc_motor_2_controller',
            output='screen'
        ),
        Node(
            package='fire_bot_pkg',
            executable='actuator_controller',
            name='actuator_controller',
            output='screen'
        ),
        Node(
            package='fire_bot_pkg',
            executable='stepper_motor_controller',
            name='stepper_motor_controller',
            output='screen'
        ),
        Node(
            package='fire_bot_pkg',
            executable='object_alignment',
            name='object_alignment',
            output='screen'
        ),

        # v4l2_camera의 노드
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            output='screen'
        ),
    ])
