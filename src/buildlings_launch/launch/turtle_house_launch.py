from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
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
    
    # Set Gazebo model path to include your models directory (CORRECT for Gazebo Classic)
    pkg_dir = get_package_share_directory('buildlings_launch')
    gazebo_model_path = os.path.join(pkg_dir, 'models')
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
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
                'z_pose': z_pose
            }.items(),
        ),
        spawn_aruco_box
    ])