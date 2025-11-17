import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Keyboard Control
    keyboard_control = [
        Node(
            package='keyboard_control',
            executable='keyboard_control_plus',
            name='keyboard_control',            
            output='screen',
            prefix="xterm -hold -e",
            parameters=[{
                "linear_v_inc": 0.1,
                "angular_v_inc": 0.3,
                "publish_topic": "/cmd_vel"
            }]   
        ),
    ]    
    

    actions=[]
    actions.extend(keyboard_control)    
    return[
        GroupAction
        (
            actions=actions
        )
    ]


def generate_launch_description():

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
            ),
        OpaqueFunction(function = launch_setup)
    ])