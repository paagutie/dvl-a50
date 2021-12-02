import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():

    dvl_a50 = launch_ros.actions.Node(
        package='dvl_a50', node_executable='dvl_a50_node', output='screen')


    return launch.LaunchDescription([
        dvl_a50,
    ])


       
