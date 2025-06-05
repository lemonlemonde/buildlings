import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FrameListener(Node):

    def __init__(self):
        super().__init__('aruco_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'ar_marker_42').get_parameter_value().string_value
        
        # 
        # TODO: load or make dict of aruco_objs 
        # aruco_dict = 

        # buffer to store most recent transforms
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

        # if SIGINT, want to save the yaml
        self.add_on_shutdown(self.cleanup)

    def cleanup(self):
        self.get_logger().info("Shutting down node... saving yaml!")
        # TODO: something

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'map'

        # now.... transform it
        try:
            # check if marker frame --> map frame transform avail
            self.get_logger().info('Found marker --> map transform!')
            t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        # wtf is a child_frame_id

        # now.......... um save it?
        # t is type TransformStamped, so....
            # std_msgs/Header header
            #     builtin_interfaces/Time stamp
            #     string frame_id
            # string child_frame_id
            # geometry_msgs/Transform transform
            #     geometry_msgs/Vector3 translation
            #         float64 x
            #         float64 y
            #         float64 z
            #     geometry_msgs/Quaternion rotation
            #         float64 x
            #         float64 y
            #         float64 z
            #         float64 w
        # sample data:
            # [aruco_listener-1] [INFO] [74.931614458]: header.stamp: builtin_interfaces.msg.Time(sec=17, nanosec=465000000)
            # [aruco_listener-1] [INFO] [74.932262520]: header.stamp.sec: 17
            # [aruco_listener-1] [INFO] [74.936147974]: header.stamp.nanosec: 465000000
            # [aruco_listener-1] [INFO] [74.937421037]: header.frame_id: map
            # [aruco_listener-1] [INFO] [74.938419781]: child_frame_id: ar_marker_42
            # [aruco_listener-1] [INFO] [74.938971969]: transform.translation.x: -2.7505846130613443
            # [aruco_listener-1] [INFO] [74.939783992]: transform.translation.y: 3.0047867045205123
            # [aruco_listener-1] [INFO] [74.940484359]: transform.translation.z: 0.11769267778904915
            # [aruco_listener-1] [INFO] [74.941230578]: transform.rotation.x: -0.4988906601839331
            # [aruco_listener-1] [INFO] [74.941749192]: transform.rotation.y: 0.5039258438834794
            # [aruco_listener-1] [INFO] [74.944168995]: transform.rotation.z: 0.49874890075564615
            # [aruco_listener-1] [INFO] [74.944729940]: transform.rotation.w: -0.4984138712381481


        self.get_logger().info(f'header.stamp: {t.header.stamp}')
        self.get_logger().info(f'header.stamp.sec: {t.header.stamp.sec}')
        self.get_logger().info(f'header.stamp.nanosec: {t.header.stamp.nanosec}')
        self.get_logger().info(f'header.frame_id: {t.header.frame_id}')
        self.get_logger().info(f'child_frame_id: {t.child_frame_id}')
        self.get_logger().info(f'transform.translation.x: {t.transform.translation.x}')
        self.get_logger().info(f'transform.translation.y: {t.transform.translation.y}')
        self.get_logger().info(f'transform.translation.z: {t.transform.translation.z}')
        self.get_logger().info(f'transform.rotation.x: {t.transform.rotation.x}')
        self.get_logger().info(f'transform.rotation.y: {t.transform.rotation.y}')
        self.get_logger().info(f'transform.rotation.z: {t.transform.rotation.z}')
        self.get_logger().info(f'transform.rotation.w: {t.transform.rotation.w}')



def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()