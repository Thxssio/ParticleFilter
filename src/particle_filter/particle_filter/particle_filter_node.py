import rclpy
from rclpy.node import Node

class ParticleFilterNode(Node):
    def __init__(self):
        super().__init__('particle_filter_node')
        self.get_logger().info('Particle Filter Node has been started.')

def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
