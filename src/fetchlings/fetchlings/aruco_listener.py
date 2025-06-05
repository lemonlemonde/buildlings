import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from ament_index_python.packages import get_package_share_directory
import os
import yaml

from fetchlings.aruco_marker import ArucoMarker, arucomarker_constructor, arucomarker_representer

from rclpy.impl import rcutils_logger


class ArucoListener(Node):

    def __init__(self):
        super().__init__('aruco_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'ar_marker_42').get_parameter_value().string_value
        
        # yaml config
        yaml.add_constructor('!ArucoMarker', arucomarker_constructor)
        yaml.add_representer(ArucoMarker, arucomarker_representer)
        
        # load or make dict of ArucoMarkers
        self.aruco_file_path = self.declare_parameter('map_dir', os.path.join(os.path.expanduser('~'), 'aruco_dict.yaml'))
        self.aruco_dict = {}
        if os.path.exists(os.path.expanduser(self.aruco_file_path.value)):
            with open(os.path.expanduser(self.aruco_file_path.value), 'r') as f:
                self.get_logger().debug(f'Loading existing ArUco dict file: {os.path.expanduser(self.aruco_file_path.value)}')
                self.aruco_dict = yaml.load(f, Loader=yaml.Loader)

        # buffer to store most recent transforms
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def cleanup(self):
        # write updated dict into yaml
        with open(os.path.expanduser(self.aruco_file_path.value), 'w') as f:
            yaml.dump(self.aruco_dict, f)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'map'

        # now.... transform it
        try:
            # check if marker frame --> map frame transform avail
            t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().debug(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
    
        # now....save it?
        # t is type TransformStamped
        #   parse in class init
        self.get_logger().info('Saving marker --> map transform!')
        obj = ArucoMarker(t)
        self.aruco_dict[obj.obj_frame_id] = obj
        



def main():
    rclpy.init()
    node = ArucoListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger = rcutils_logger.RcutilsLogger(name="rc_node")
        logger.info('[aruco_listener] KeyboardInterrupt caught. Exiting...')
    finally:
        # save yaml
        logger.info('[aruco_listener] Saving yaml before exiting...')
        node.cleanup()
        rclpy.shutdown()