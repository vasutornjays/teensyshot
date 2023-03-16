import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as Lc
from launch.actions import DeclareLaunchArgument


def launch_setup(context, *args, **kwargs):
    
    teensyshot_node = Node(
        package='teensyshot',
        executable='teensyshot_host_ros',
        namespace='xplorer_mini',
        name='teensyshot_host_ros',
        output='screen',
        emulate_tty=True,
    )

    # return [velocity_contoller, position_contoller, thruster_manager]
    return [teensyshot_node]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
