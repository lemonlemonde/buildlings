import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from buildlings_msgs.msg import WorldState



class AgentNode1(Node):

    def __init__(self):
        super().__init__('agent_1')
        self.subscription = self.create_subscription(
            WorldState,
            'state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('The goal coord: "%s"' % msg.goal_coord)
        self.get_logger().info('The grid state: "%s"' % msg.grid_state)
        self.get_logger().info('The agent states: "%s"' % msg.agent_states)


def main(args=None):
    rclpy.init(args=args)

    node = AgentNode1()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()