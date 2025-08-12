from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():


    # Node for the detection logic
    detection_node = Node(
        package='pi_patrol',
        executable='detection_node',
        name='detection_node',
        output='screen',
        emulate_tty=True, # Ensure logger output is displayed
    )

    # Camera node
    camera_node = Node(
        package='pi_patrol',
        executable='camera_node',
        name='camera_node',
        output='screen',
        emulate_tty=True,
    )

    # Recorder node
    recorder_node = Node(
        package='pi_patrol',
        executable='recorder_node',
        name='recorder_node',
        output='screen',
        emulate_tty=True,
    )

    # Telegram notifier node
    telegram_notifier_node = Node(
        package='pi_patrol',
        executable='telegram_notifier_node',
        name='telegram_notifier_node',
        output='screen',
        emulate_tty=True,
    )

    # # Node for motor control
    # motor_control_node = Node(
    #     package='pi_patrol',
    #     executable='motor_control_node',
    #     name='motor_control_node',
    #     output='screen'
    # )

    # # Node for following the target
    # follow_target_node = Node(
    #     package='pi_patrol',
    #     executable='follow_target_node',
    #     name='follow_target_node',
    #     output='screen',
    #     emulate_tty=True,
    # )

    # # Get the share directory for the pi_patrol_sim package
    #pi_patrol_sim_share_dir = get_package_share_directory('pi_patrol_sim')

    # # Include the Gazebo launch file
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    #     ),
    #     launch_arguments={'world': LaunchConfiguration('world')}.items()
    # )

    # # Declare launch arguments
    # world_arg = DeclareLaunchArgument(
    #     'world',
    #     default_value=os.path.join(pi_patrol_sim_share_dir, 'worlds', 'pi_patrol_world.world'),
    #     description='Path to the Gazebo world file'
    # )


    # # Node for spawning the robot
    # spawn_robot_node = Node(
    #     package='pi_patrol_sim',
    #     executable='spawn_robot',
    #     name='spawn_robot',
    #     output='screen'
    # )
    return LaunchDescription([
        camera_node,
        detection_node,
        recorder_node,
        telegram_notifier_node,
        # motor_control_node,
        # follow_target_node,
        # gazebo_launch,
        # sim_launch,
    ])
