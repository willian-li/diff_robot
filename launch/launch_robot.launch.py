import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='diff_robot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    controller_params = os.path.join(
        get_package_share_directory('diff_robot'), # <-- Replace with your package name
        'config',
        'my_controllers.yaml'
        )


    # Delete the Gazebo stuff and replace it with this
    controller_manager = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_params],
            )
    
    delayed_controller_manager = TimerAction(period=3.0,actions=[controller_manager])
    

    diff_drive_spawner = Node(
                        package="controller_manager",
                        executable="spawner.py",
                        arguments=["diff_cont"],
                    )

    joint_broad_spawner = Node(
                        package="controller_manager",
                        executable="spawner.py",
                        arguments=["joint_broad"],
                    )

    delayed_diff_drive_spawner = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[diff_drive_spawner],
            )
        )
    delayed_joint_broad_spawner = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[joint_broad_spawner],
            )
        )
    

    # Launch them all!
    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])
