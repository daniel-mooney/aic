from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    debug_node = Node(
        package='superviser',
        executable='debug_rrt_node',
        name='debug_rrt_node',
        output='screen',
        emulate_tty=True,
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("superviser"),
                "rviz",
                "config.rviz",
            ])
        ],
        output='screen'
    )

    return LaunchDescription([
        rviz,
        debug_node,
    ])
