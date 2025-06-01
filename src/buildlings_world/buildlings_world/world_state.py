import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from utils import BlockState
# custom worldstate message
from buildlings_msgs.msg import WorldState

class WorldState(Node):
    def __init__(self):
        # init basic node with name
        super().__init__('world_state')
        
        # make starting grid
        self.grid_size = (10, 10)
        self.goal_coord = (7, 9)
        temp_grid = [
            "1111110011",
            "1111110011",
            "3100000011",
            "1100000011",
            "3100000011",
            "1100000011",
            "3100000011",
            "1100000014",
            "1111000011",
            "1111000011",
        ]
        self.grid = [[BlockState(int(c)) for c in row] for row in temp_grid]
        
        # publish to topic (msg type, name, queue size)
        self.publisher_ = self.create_publisher(WorldState, 'state', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = WorldState()
        msg.goal_coord = self.goal_coord
        msg.grid_state = [[state.value for state in row] for row in self.grid]
        # TODO: implement later...
        msg.agent_states = []
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.grid_state)


def main(args=None):
    rclpy.init(args=args)

    node = WorldState()

    # starts timer, which triggers the timer_callback
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()