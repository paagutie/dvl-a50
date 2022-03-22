import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    launch.actions.DeclareLaunchArgument('ip_address', default_value='192.168.194.95'),
    dvl_a50 = launch_ros.actions.Node(
        package='dvl_a50', 
        node_executable='dvl_a50_node', 
        parameters=[{'dvl_ip_address': launch.substitutions.LaunchConfiguration('ip_address')}],
        output='screen')


    return launch.LaunchDescription([
        dvl_a50,
    ])


       
