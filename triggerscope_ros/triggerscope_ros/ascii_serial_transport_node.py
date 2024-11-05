import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
import serial


class AsciiSerialTransportNode(Node):

    def __init__(self):
        super().__init__('asciiserialtransport')
        self.subscription = self.create_subscription(
            String,
            'to_serial',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(String, 'from_serial', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Initialize serial port
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)
        self.get_logger().info('AsciiSerialTransport node initialized')

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if not msg.data.endswith('\n'):
            msg.data += '\n'
        # Write to serial port
        self.serial_port.write(msg.data.encode())

    def read_from_serial(self):
        line = self.serial_port.readline().decode('utf-8').strip()
        if line:
            msg = String()
            msg.data = line
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)

    def timer_callback(self):
        self.read_from_serial()
        

def main(args=None):

    rclpy.init(args=args)


    try:
        asciiserialtransport = AsciiSerialTransportNode()
        rclpy.spin(asciiserialtransport)
    except ExternalShutdownException:
        asciiserialtransport.get_logger.error('External shutdown')
    except KeyboardInterrupt:
        asciiserialtransport.get_logger.error('Keyboard interrupt')
    except Exception as e:
        asciiserialtransport.get_logger.error('An error occurred: %s' % e)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    finally:
        asciiserialtransport.destroy_node()
        
    rclpy.shutdown()
if __name__ == '__main__':
    main()
