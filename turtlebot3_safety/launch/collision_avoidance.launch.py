from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='turtlebot3_safety',
            executable='collision_avoidance',
            name='collision_avoidance_node',
            emulate_tty=True,
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='odometry_rviz',
            arguments=['-d', [FindPackageShare("turtlebot3_safety"), '/rviz', '/turtlebot3_safety.rviz',]]
        ),
    ])
