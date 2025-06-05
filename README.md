# What is this

Fun project..? where little robots in ROS2 are able to reach goal locations in a 2D grid world by creating bridges. Hopefully.

I made a [Docker container template for ROS2 (humble) and Gazebo](https://github.com/lemonlemonde/gazebo-ros2-docker-template) that this is built on. 

## Quick Start
1. Set up with the template instructions (above)
2. Clone this?
3. Set up with [ROS2](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) 
4. Set up your bashrc (below)
5. `ros2 launch fetchlings turtle_house_launch.py num_robots:=5 map_dir:=/home/mousey/fetchlings/aruco_dict.yaml` (cannot be in virtual env???)
  - `map_dir` will default to `~/aruco_dict.yaml` if unspecified
  - `num_robots` is 1-6 for now
6. ...


## .bashrc
These should be the only things convenient to add to your `.bashrc`.
Everything else is set up in the launch file.
```bashrc
# Source ROS 2 Humble setup (adjust if different)
source /opt/ros/humble/setup.bash

# Load your own workspace if it exists
if [ -f ~/fetchlings/install/setup.bash ]; then
  source ~/fetchlings/install/setup.bash
fi
```

## Dependencies
`pip install scipy opencv-contrib-python catkin_pkg "empy<4" "lark-parser>=0.11,<1.2"`

There are also some git submodules, and they should all be working, but if at any time need to update them...

```shell
# initial clone
git submodule update --init --recursive

# updates
git submodule update --remote --merge
```