# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace, Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Get the launch directory of gmapping
    # slam_gmapping_dir = get_package_share_directory("slam_gmapping")
    # slam_gmapping_launch_dir = os.path.join(slam_gmapping_dir, "launch")

    # Get the launch directory of map_merge
    map_merge_dir = get_package_share_directory("multirobot_map_merge")
    map_merge_launch_dir = os.path.join(map_merge_dir, "launch", "tb3_simulation")
    

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    slam_toolbox = LaunchConfiguration("slam_toolbox")
    slam_gmapping = LaunchConfiguration("slam_gmapping")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False", description="Whether run a SLAM"
    )
    declare_slam_toolbox_cmd = DeclareLaunchArgument(
        "slam_toolbox", default_value="True", description="Whether run a SLAM toolbox"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", description="Full path to map yaml file to load"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    # collapsed slam_launch.py
        # https://github.com/ros-navigation/navigation2/blob/0cdf4f44c7a4171bfa4c6eb885d46c3c19f26a87/nav2_bringup/bringup/launch/slam_launch.py
    # slam_launch.py
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    # TODO: collapse this launch file too?
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')
    lifecycle_nodes = ['map_saver']
    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'use_sim_time': use_sim_time}.items())
    start_map_saver_server_cmd = Node(
        package='nav2_map_server',
        node_executable='map_saver_server',
        output='screen',
        parameters=[params_file])
    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            node_executable='lifecycle_manager',
            node_name='lifecycle_manager_slam',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
    
    # Collapsed slam_toolbox.py
    slam_params_file = LaunchConfiguration("slam_params_file")
    remappings = [
        ("/map", "map"),
        ("/scan", "scan"),
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(
            get_package_share_directory("slam_toolbox"),
            "config",
            "mapper_params_online_sync.yaml",
        ),
        description="Full path to the ROS2 parameters file to use for the slam_toolbox node",
    )

    start_sync_slam_toolbox_node = Node(
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        remappings=remappings,
    )
    
    
    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(launch_dir, "slam_launch.py")
            #     ),
            #     condition=IfCondition(
            #         PythonExpression(
            #             [slam, " and ", slam_toolbox, " and not ", slam_gmapping]
            #         )
            #     ),
            #     launch_arguments={
            #         "namespace": namespace,
            #         "use_sim_time": use_sim_time,
            #         "autostart": autostart,
            #         "params_file": params_file,
            #     }.items(),
            # ),

            # Collapsed slam_launch.py
            start_slam_toolbox_cmd,
            start_map_saver_server_cmd,
            start_lifecycle_manager_cmd,

            
            
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(map_merge_launch_dir, "slam_toolbox.py")
            #     ),
            #     condition=IfCondition(
            #         PythonExpression(
            #             [slam, " and ", slam_toolbox, " and not ", slam_gmapping]
            #         )
            #     ),
            #     launch_arguments={
            #         "use_sim_time": use_sim_time,
            #     }.items(),
            # ),
            
            # Collapsed slam_toolbox.py
            start_sync_slam_toolbox_node,
            
            
            
            
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "localization_launch.py")
                ),
                condition=IfCondition(PythonExpression(["not ", slam])),
                launch_arguments={
                    "namespace": namespace,
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_lifecycle_mgr": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_lifecycle_mgr": "false",
                    "map_subscribe_transient_local": "true",
                }.items(),
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_slam_toolbox_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_slam_params_file_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)
    # ld.add_action(slam_gmapping_cmd)

    return ld
