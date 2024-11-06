import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import triggerscope_msgs.srv as srv 
import serial

# Map of voltage ranges to their corresponding min and max values
# This is used to convert the range enum to the actual voltage range
# that the Triggerscope expects
# These are constants, do not modify
ANALOG_VOLTAGE_RANGES = {
    srv.SetAnalogRange_Request.VOLTAGE_RANGE_0_TO_5: (0., 5.),
    srv.SetAnalogRange_Request.VOLTAGE_RANGE_0_TO_10: (0., 10.),
    srv.SetAnalogRange_Request.VOLTAGE_RANGE_NEGATIVE_5_TO_5: (-5., 5.),
    srv.SetAnalogRange_Request.VOLTAGE_RANGE_NEGATIVE_10_TO_10: (-10., 10.),
    srv.SetAnalogRange_Request.VOLTAGE_RANGE_NEGATIVE_2_TO_2: (-2., 2.),
}

class TriggerscopeServerNode(Node):
    """
    Node that interfaces a Triggerscope device with ROS2
    """
    
    def __init__(self):
        """
        Initialize the TriggerscopeServerNode
        """
        
        # Map of analog voltage ranges keyed by channel
        self.analog_ranges = {}
        
        # Initialize the Base Node Class
        super().__init__('triggerscopeserver')

        # Create services
        self.srv = self.create_service(srv.SetDigitalOut, 'set_digital_out', self.set_digital_output_callback)
        self.srv = self.create_service(srv.SetAnalogRange, 'set_analog_range', self.set_analog_range_callback)

        # Initialize serial port
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.2)
        self.get_logger().info('TriggerscopeServer node initialized')


    def set_digital_output_callback(self, request:srv.SetAnalogOut_Request, response:srv.SetAnalogOut_Response):
        self.get_logger().debug('Received request to set digital output %d to %d' % (request.channel, request.state))
        outstring = f"SDO{int(request.channel)}-{int(request.state)}"
        self.serial_port.write((outstring + '\n').encode("ascii"))
        
        line = self.serial_port.readline().strip().decode('ascii')

        if line and (line == '!' + outstring):
            self.get_logger().debug('Successfully set digital output %d to %d' % (request.channel, request.state))
            response.success = True
        else:
            self.get_logger().error('Failed to set digital output %d to %d' % (request.channel, request.state))
            self.get_logger().error('Received: %s' % line)
            response.success = False
            
        return response

    def set_analog_range_callback(self, request:srv.SetAnalogOut_Request, response:srv.SetAnalogOut_Response):
        self.get_logger().debug('Received request to set analog range %d to %d' % (request.channel, request.range))
        outstring = f"SAR{int(request.channel)}-{int(request.range)}"
        self.serial_port.write((outstring + '\n').encode("ascii"))
        
        line = self.serial_port.readline().strip().decode('ascii')

        if line and (line == '!' + outstring):
            self.get_logger().debug('Successfully set analog range %d to %d' % (request.channel, request.range))
            response.success = True
        else:
            self.get_logger().error('Failed to set analog range %d to %d' % (request.channel, request.range))
            self.get_logger().error('Received: %s' % line)
            response.success = False
        
        if response.success:
            self.analog_ranges[request.channel] = ANALOG_VOLTAGE_RANGES[request.range]
            
        return response
        
    def do_serial_comms(self, command):
        self.serial_port.write((command + '\n').encode("ascii"))
        line = self.serial_port.readline().strip().decode('ascii')
        return line
        

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
