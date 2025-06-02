from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    turtle_pkg = get_package_share_directory('turtlebot3_gazebo')

    world_path = os.path.join(
        turtle_pkg, 
        'worlds', 
        'turtlebot3_house.world'
    )

    # spawn turtle here
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')

    # all the preconfigged launch files
    gzserver_launch = os.path.join(gazebo_pkg, 'launch', 'gzserver.launch.py')
    gzclient_launch = os.path.join(gazebo_pkg, 'launch', 'gzclient.launch.py')
    robot_state_publisher_launch = os.path.join(turtle_pkg, 'launch', 'robot_state_publisher.launch.py')
    spawn_turtlebot_launch = os.path.join(turtle_pkg, 'launch', 'spawn_turtlebot3.launch.py')


    # format the expected launch description
    return LaunchDescription([
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
                'z_pose': z_pose
            }.items(),
        ),

        # # Start your teleop/drive/control node here if you want
        # Node(
        #     package='my_custom_control_pkg',
        #     executable='my_control_node',
        #     name='my_control_node',
        #     output='screen'
        # ),
    ])
