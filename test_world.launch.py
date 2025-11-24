from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Locate the package
    pkg_path = get_package_share_directory('my_world')

    # Path to your SDF world
    world_path = os.path.join(pkg_path, 'worlds', 'TestWorld.sdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose', world_path,
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        )
    ])
