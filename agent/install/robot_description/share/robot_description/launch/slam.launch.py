import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_description').find('robot_description')
    slam_params = os.path.join(pkg_share, 'config/slam_online_sync.yaml')

    sync_slam_toolbox_node = launch_ros.actions.Node(
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                            description='Flag to enable use_sim_time'),
        sync_slam_toolbox_node
    ])