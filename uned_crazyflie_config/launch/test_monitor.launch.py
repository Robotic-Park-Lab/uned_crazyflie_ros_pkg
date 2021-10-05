import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='uned_crazyflie_driver', executable='crazyflie_driver', output='screen',
            name='cf_driver'),
        launch_ros.actions.Node(
            package='uned_vicon_bridge', executable='vicon_tracker', output='screen',
            name='vicon_tracker'),
        launch_ros.actions.Node(
            package='uned_crazyflie_controllers', executable='periodic_pid_position_controller', output='screen',
            name='position_controller'),
    ])
