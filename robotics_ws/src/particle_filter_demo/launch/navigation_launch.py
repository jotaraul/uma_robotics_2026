import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetLaunchConfiguration,IncludeLaunchDescription,SetEnvironmentVariable,OpaqueFunction,GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution
from launch_ros.parameter_descriptions import ParameterFile
from ros2launch.api import get_share_file_path_from_package

#===========================
def launch_arguments():
    return [
        DeclareLaunchArgument("", default_value=""),
   ]
#==========================

def launch_setup(context, *args, **kwargs):

    configured_params = ParameterFile(
        get_share_file_path_from_package(package_name="particle_filter_demo", file_name="params.yaml")
    )

    navigation_nodes = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[
                {"yaml_filename": get_share_file_path_from_package(package_name="particle_filter_demo", file_name="map.yaml")},
                {"frame_id": "map"},
            ],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
        ),
        # LIFECYCLE MANAGER
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[
                    {"autostart": True},
                    {
                        "node_names": [
                            "map_server",
                            "amcl"
                        ]
                    },
            ],
        ),
    ]

    # To create the map, return this instead of the navigation_nodes list
    slam = LifecycleNode(
        autostart=True,
        namespace='',
        parameters=[configured_params],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        prefix="xterm -hold -e",
    )

    nodes = []
    nodes.extend(navigation_nodes)

    return nodes


def generate_launch_description():

    launch_description = [
       # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
   ]
   
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
   
    return  LaunchDescription(launch_description)