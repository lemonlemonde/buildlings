
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # shit i need
        # - gzserver
        # - gzclient
        # - multi_robot_launch.py for passing in arg
            # - this needs to be modified with multi_robot_launch_with_nav.py
                # - spawn n robots
                # - SLAM
                # - nav2
                # - explore_lite
        # - aruco box
    
    cur_pkg = get_package_share_directory('fetchlings')
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    turtle_pkg = get_package_share_directory('turtlebot3_gazebo')
    nav2_pkg = get_package_share_directory('turtlebot3_navigation2')
    slam_tlbx_pkg = get_package_share_directory('slam_toolbox')
    aruco_tracker_pkg = get_package_share_directory('turtlebot3_aruco_tracker')
    world_path = os.path.join(
        turtle_pkg, 
        'worlds', 
        'turtlebot3_house.world'
    )

    declare_map_dir_arg = DeclareLaunchArgument(
        "map_dir", default_value=os.path.join(os.path.expanduser('~'), 'aruco_dict.yaml'), description="Directory and file name to save maps and markers"
    )
    declare_num_robots_arg = DeclareLaunchArgument(
        "num_robots", default_value='2', description="Number of robots to spawn"
    )
    
    # # spawn turtle here
    # x_pose = LaunchConfiguration('x_pose', default='-4.5')
    # y_pose = LaunchConfiguration('y_pose', default='3.0')
    # z_pose = LaunchConfiguration('z_pose', default='0.1')
    
    # all the preconfigged launch files
    gzserver_launch = os.path.join(gazebo_pkg, 'launch', 'gzserver.launch.py')
    gzclient_launch = os.path.join(gazebo_pkg, 'launch', 'gzclient.launch.py')
    spawn_turtlebots_launch = os.path.join(cur_pkg, 'launch', 'multi_robot_launch.py')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation2.launch.py')
    slam_tlbx_launch = os.path.join(slam_tlbx_pkg, 'launch', 'online_async_launch.py')
    aruco_tracker_launch = os.path.join(aruco_tracker_pkg, 'launch', 'turtlebot3_aruco_tracker.launch.py')

    # params files
    nav2_params_path = os.path.join(cur_pkg, 'params', 'nav2_waffle_pi.yaml')
    explore_params_path = os.path.join(cur_pkg, "params", "explore_params.yaml")

    
    # Set Gazebo model path to include your models directory (for Gazebo Classic)
    gazebo_model_path = os.path.join(cur_pkg, 'models')
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
                  '-x', '-4.0', '-y', '3.0', '-z', '1.0'],  # Moved away from turtle spawn
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
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]
    )

    # aruco listener
    aruco_listener_node = Node(
        package="fetchlings",
        executable="aruco_listener",
        name="aruco_listener",
        output="screen",
        parameters=[{
            'map_dir': LaunchConfiguration('map_dir')
        }]
    )
    


    
    # format the expected launch description
    return LaunchDescription([
        # Set the environment variable FIRST
        set_gazebo_model_path,
        set_turtlebot3_model,
        set_ros_domain_id,

        # map dir arg
        declare_map_dir_arg,
        declare_num_robots_arg,
        
        # launch gazebo server via their own launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzserver_launch),
            launch_arguments={'world': world_path}.items(),
        ),
        # gazebo client via their own launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzclient_launch),
        ),

        # spawn aruco box model in env
        spawn_aruco_box,

        # ==================== ALL OF BELOW PRESUMABLY
        # ==================== REPLACED BY MULTI_ROBOT_LAUNCH.PY?????
        # # actual turtles via their own launch file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(spawn_turtlebots_launch),
        #     launch_arguments={
        #         'num_robots': LaunchConfiguration('num_robots'),
        #         'use_sim_time': 'true',
        #     }.items(),
        # ),


        # # slam
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(slam_tlbx_launch),
        #     launch_arguments={
        #         'use_sim_time': 'true'
        #     }.items()
        # ),

        # # nav2
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(nav2_launch),
        #     launch_arguments={
        #         'use_sim_time': 'true',
        #         'params_file': nav2_params_path
        #     }.items()
        # ),

        # # m-explore-ros2
        # explore_node,
        # ==================== ALL OF ABOVE PRESUMABLY
        # ==================== REPLACED BY MULTI_ROBOT_LAUNCH.PY?????

        # aruco tracker
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(aruco_tracker_launch),
            launch_arguments={
                'marker_size': '0.2'
            }.items()
        ),

        # listens and saves marker yaml
        aruco_listener_node

    ])