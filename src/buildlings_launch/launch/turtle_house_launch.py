
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    cur_pkg = get_package_share_directory('buildlings_launch')
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    turtle_pkg = get_package_share_directory('turtlebot3_gazebo')
    nav2_pkg = get_package_share_directory('turtlebot3_navigation2')
    slam_tlbx_pkg = get_package_share_directory('slam_toolbox')
    explore_pkg = get_package_share_directory('explore_lite')
    world_path = os.path.join(
        turtle_pkg, 
        'worlds', 
        'turtlebot3_house.world'
    )
    
    # spawn turtle here
    x_pose = LaunchConfiguration('x_pose', default='-4.5')
    y_pose = LaunchConfiguration('y_pose', default='3.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')
    
    # all the preconfigged launch files
    gzserver_launch = os.path.join(gazebo_pkg, 'launch', 'gzserver.launch.py')
    gzclient_launch = os.path.join(gazebo_pkg, 'launch', 'gzclient.launch.py')
    robot_state_publisher_launch = os.path.join(turtle_pkg, 'launch', 'robot_state_publisher.launch.py')
    spawn_turtlebot_launch = os.path.join(turtle_pkg, 'launch', 'spawn_turtlebot3.launch.py')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation2.launch.py')
    slam_tlbx_launch = os.path.join(slam_tlbx_pkg, 'launch', 'online_async_launch.py')

    # params files
    nav2_params_path = os.path.join(cur_pkg, 'params', 'nav2_waffle_pi.yaml')
    explore_params_path = os.path.join(cur_pkg, "params", "explore_params.yaml")

    
    # Set Gazebo model path to include your models directory (CORRECT for Gazebo Classic)
    pkg_dir = get_package_share_directory('buildlings_launch')
    gazebo_model_path = os.path.join(pkg_dir, 'models')
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path + ':' + "/opt/ros/humble/share/turtlebot3_gazebo/models/" + ":" + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    # set turtlebot3 model for camera
    set_turtlebot3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value="waffle_pi"
    )

    # set ROS_DOMAIN_ID!
    set_ros_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value="30"
    )

    spawn_aruco_box = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'aruco_box', 
                  '-database', 'aruco_box',
                  '-x', '2.0', '-y', '2.0', '-z', '1.0'],  # Moved away from turtle spawn
        output='screen'
    )

    # m-explore-ros2
    explore_node = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        output="screen",
        parameters=[
            explore_params_path,  # base config
            {
                'use_sim_time': True,
                'potential_scale': 3.0, # decrease, less aggressive movement
                'gain_scale': 2.5, # increase, prioritize frontiers farther away
                'min_frontier_size': 0.1, # decrease, explore smaller frontiers
                'progress_timeout': 80.0, # time for recovery if fall
                'planner_frequency': 1.0, # increase freq Hz for planner
            }
            # /**: default
            #   ros__parameters:
            #     robot_base_frame: base_link
            #     costmap_topic: map
            #     costmap_updates_topic: map_updates
            #     visualize: true
            #     planner_frequency: 0.25
            #     progress_timeout: 30.0
            #     potential_scale: 3.0
            #     orientation_scale: 0.0
            #     gain_scale: 1.0
            #     transform_tolerance: 0.3
            #     min_frontier_size: 0.75
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]
    )

    # explore_node = Node(
    #     package='explore_lite',
    #     executable='explore',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'planner_frequency': 0.33,
    #         'progress_timeout': 30.0,  # Increase if robot gets stuck
    #         'potential_scale': 3.0,
    #         'gain_scale': 1.0,
    #         'min_frontier_size': 0.4,  # Lower = more sensitive
    #         'visualize': True
    #     }],
    #     output='screen'
    # )
    # # 10 sec?
    # delayed_explore_node = TimerAction(
    #     period=70.0, 
    #     actions=[explore_node]
    # )
    
    # format the expected launch description
    return LaunchDescription([
        # Set the environment variable FIRST
        set_gazebo_model_path,
        set_turtlebot3_model,
        set_ros_domain_id,
        
        # launch gazebo server via their own launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzserver_launch),
            launch_arguments={'world': world_path}.items(),
        ),
        # gazebo client via their own launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzclient_launch),
        ),
        # turtle publisher node launch via their own launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_publisher_launch),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
        # actual turtle via their own launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_turtlebot_launch),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
                'z_pose': z_pose,
                'use_sim_time': 'true',
            }.items(),
        ),
        spawn_aruco_box,

        # slam
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_tlbx_launch),
            launch_arguments={
                'use_sim_time': 'true'
            }.items()
        ),

        # nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params_path
            }.items()
        ),

        # m-explore-ros2
        explore_node

    ])