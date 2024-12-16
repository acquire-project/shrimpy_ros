import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Image


class PhaseReconstructionNode(Node):

    def __init__(self):
        super().__init__('phase_reconstruction_node')
        self.sub = self.create_subscription(Image, 'image_raw', self.image_callback, 10)
        self.image_queue = []
        self.get_logger().info("Phase reconstruction node started")
        
    def image_callback(self, msg: Image):
        self.image_queue.append(msg)
        self.get_logger().info(f"Received image {msg.header.seq}, I now have {len(self.image_queue)} images")

def main(args=None):
    rclpy.init(args=args)

    node = PhaseReconstructionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
