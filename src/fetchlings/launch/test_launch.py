
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    cur_pkg = get_package_share_directory('fetchlings')

    declare_map_dir_arg = DeclareLaunchArgument(
        "map_dir", default_value=os.path.join(os.path.expanduser('~'), 'aruco_dict.yaml'), description="Directory and file name to save maps and markers"
    )

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

        declare_map_dir_arg,
        
        aruco_listener_node
    ])