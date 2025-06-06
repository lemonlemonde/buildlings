#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool, HyunGyu Kim

# ^^ original authors
# modified version
# pass in `num_robots` launch arg
# maximum of 6!!! (otherwise need to increase `pose` list)
# also adding SLAM, nav2, explore_lite stuff based on the m-explore-ros2 multi-robot launching and merging script

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node

#             # - this needs to be merged with multi_robot_launch_with_nav.py
# - spawn n robots
# - SLAM
# - nav2
# - explore_lite


def launch_setup(context, *args, **kwargs):
    TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]
    model_folder = "turtlebot3_" + TURTLEBOT3_MODEL
    number_of_robots = int(LaunchConfiguration("num_robots").perform(context))
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    namespace = "TB3"

    # max 6 robots for now
    pose = [[-2, -0.5], [0.5, -2], [2.0, 0.5], [-0.5, 2], [-4.5, 3.0], [-0.5, 0.0]]
    urdf_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "models",
        model_folder,
        "model.sdf",
    )
    save_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "models", model_folder, "tmp"
    )
    
    cur_pkg = get_package_share_directory("fetchlings")
    turtlebot3_launch_dir = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "launch")
    
    # slam
    slam_tlbx_pkg = get_package_share_directory("slam_toolbox")
    slam_tlbx_launch = os.path.join(slam_tlbx_pkg, "launch", "online_async_launch.py")
    
    # explore params
    explore_params_path = os.path.join(cur_pkg, "params", "explore_params.yaml")
    
    # nav2
    nav2_pkg = get_package_share_directory("turtlebot3_navigation2")
    nav2_launch = os.path.join(nav2_pkg, "launch", "navigation2.launch.py")
    nav2_params_path = os.path.join(cur_pkg, "params", "nav2_waffle_pi.yaml")

    # nav2-bringup
    bringup_dir = get_package_share_directory("nav2_bringup")
    bringup_launch_dir = os.path.join(bringup_dir, "launch")
    
    # multirobot_map_merge launch dir
    #   they have modified launch files?
    map_merge_dir = get_package_share_directory("multirobot_map_merge")
    launch_dir_map_merge = os.path.join(map_merge_dir, "launch", "tb3_simulation")


    robo_cmd_list = []

    for count in range(number_of_robots):
        robo_namespace = f"{namespace}_{count+1}"
        
        state_pub_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_launch_dir, "robot_state_publisher.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "frame_prefix": robo_namespace,
            }.items(),
        )

        tree = ET.parse(urdf_path)
        root = tree.getroot()
        for odom_frame_tag in root.iter("odometry_frame"):
            odom_frame_tag.text = f"{robo_namespace}/odom"
        for base_frame_tag in root.iter("robot_base_frame"):
            base_frame_tag.text = f"{robo_namespace}/base_footprint"
        for scan_frame_tag in root.iter("frame_name"):
            scan_frame_tag.text = f"{robo_namespace}/base_scan"
        urdf_modified = ET.tostring(tree.getroot(), encoding="unicode")
        urdf_modified = '<?xml version="1.0" ?>\n' + urdf_modified
        with open(f"{save_path}{count+1}.sdf", "w") as file:
            file.write(urdf_modified)


        # spawn turtlebot3
        spawn_turtlebot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_launch_dir, "multi_spawn_turtlebot3.launch.py")
            ),
            launch_arguments={
                "x_pose": str(pose[count][0]),
                "y_pose": str(pose[count][1]),
                "robot_name": f"{TURTLEBOT3_MODEL}_{count+1}",
                "namespace": robo_namespace,
                "sdf_path": f"{save_path}{count+1}.sdf",
            }.items(),
        )

        # explore_lite (m-explore-ros2)
        explore_node = Node(
            package="explore_lite",
            executable="explore",
            name="explore_node",
            namespace=robo_namespace,
            output="screen",
            parameters=[
                explore_params_path,  # base config
                {
                    "use_sim_time": True,
                    "potential_scale": 3.0,  # decrease, less aggressive movement
                    "gain_scale": 2.5,  # increase, prioritize frontiers farther away
                    "min_frontier_size": 0.1,  # decrease, explore smaller frontiers
                    "progress_timeout": 80.0,  # time for recovery if fall
                    "planner_frequency": 1.0,  # increase freq Hz for planner
                },
            ],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        )

        # slam
        slam_tlbx_launch_action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_tlbx_launch),
            launch_arguments={"use_sim_time": "true"}.items(),
        )

        # nav2
        nav2_launch_action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "use_sim_time": "true",
                "params_file": nav2_params_path,
            }.items(),
        )

        robo_cmd_list.append(
            GroupAction(
                [
                    PushRosNamespace(robo_namespace),
                    state_pub_cmd,
                    spawn_turtlebot_cmd,
                    explore_node,
                    slam_tlbx_launch_action,
                    nav2_launch_action
                ]
            )
        )

    actions = []
    # append spawn cmds
    actions.extend(robo_cmd_list)

    # shutdown
    actions.append(
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=lambda event, context: [
                    os.remove(f"{save_path}{count+1}.sdf")
                    for count in range(number_of_robots)
                ]
            )
        )
    )

    # opaque function will add these into the launch description
    return actions


def generate_launch_description():

    declare_num_robots_arg = DeclareLaunchArgument(
        "num_robots", default_value="2", description="Number of robots to spawn"
    )

    ld = LaunchDescription()
    # declare launch arg first
    ld.add_action(declare_num_robots_arg)

    # then opaque function with everything that relies on declared launch arg
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
