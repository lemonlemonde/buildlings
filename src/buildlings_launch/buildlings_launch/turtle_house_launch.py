from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gazebo_pkg = get_package_share_directory('ros_gz_sim')
    turtle_pkg = get_package_share_directory('turtlebot3_gazebo')

    world_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 
        'models', 
        'turtlebot3_house',
        'model.sdf'
    )
    model_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_burger',
        'model.sdf'
    )

    # format the expected launch description
    return LaunchDescription([
        # launch gazebo via `ros_gz_sim` package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gz_sim_launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world_path}'}.items()
        ),

        # add turtle
            # using same `ros_gz_sim` package, create/spawn a thing!
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'turtlebot3',
                '-x', '0', '-y', '0', '-z', '0.1',
                '-file', os.path.join(turtle_pkg, 'models', 'turtlebot3_burger', 'model.sdf')
            ],
            output='screen'
        ),

        # # Start your teleop/drive/control node here if you want
        # Node(
        #     package='my_custom_control_pkg',
        #     executable='my_control_node',
        #     name='my_control_node',
        #     output='screen'
        # ),
    ])
