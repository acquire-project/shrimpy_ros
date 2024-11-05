import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from triggerscope_msgs.srv import SetDigitalOut
import serial


class TriggerscopeServerNode(Node):

    def __init__(self):
        super().__init__('triggerscopeserver')

        self.srv = self.create_service(SetDigitalOut, 'set_digital_out', self.set_digital_output_callback)

        # Initialize serial port
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.2)
        self.get_logger().info('TriggerscopeServer node initialized')


    def set_digital_output_callback(self, request, response):
        self.get_logger().debug('Received request to set digital output %d to %d' % (request.channel, request.state))
        outstring = f"SDO{int(request.channel)}-{int(request.state)}"
        self.serial_port.write((outstring + '\n').encode("ascii"))
        
        line = self.serial_port.readline().strip().decode('ascii')

        if line and (line == '!' + outstring):
            self.get_logger().debug('Successfully set digital output %d to %d' % (request.channel, request.state))
            response.success = True
        else:
            self.get_logger().error('Failed to set digital output %d to %d' % (request.channel, request.state))
            self.get_logger().error('Received: \'%s\'' % line)
            response.success = False
            
        return response

    def timer_callback(self):
        self.read_from_serial()
        

def main(args=None):

    rclpy.init(args=args)


    try:
        triggerscopeserver = TriggerscopeServerNode()
        rclpy.spin(triggerscopeserver)
    except ExternalShutdownException:
        triggerscopeserver.get_logger.error('External shutdown')
    except KeyboardInterrupt:
        triggerscopeserver.get_logger.error('Keyboard interrupt')
    except Exception as e:
        triggerscopeserver.get_logger.error('An error occurred: %s' % e)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    finally:
        triggerscopeserver.destroy_node()
        
    rclpy.shutdown()
if __name__ == '__main__':
    main()
