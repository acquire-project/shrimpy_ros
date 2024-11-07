import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import triggerscope_msgs.srv as srv 
import serial
import math



class TriggerscopeServerNode(Node):
    
    """
    Node that interfaces a Triggerscope device with ROS2
    """
    
    
    # Static Constants
    
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

    # Min and max values for the DAC
    DAC_MIN = 0
    DAC_MAX = 65535 

    def __init__(self):
        """
        Initialize the TriggerscopeServerNode
        """
        
        # Map of analog voltage ranges keyed by channel
        self.analog_ranges = {}
        
        # Initialize the Base Node Class
        super().__init__('triggerscopeserver')

        # Create services
        self.srv = {}
        self.srv['set_digital_out'] =  self.create_service(srv.SetDigitalOut, 'set_digital_out', self.set_digital_output_callback)
        self.srv['set_analog_range'] = self.create_service(srv.SetAnalogRange, 'set_analog_range', self.set_analog_range_callback)
        self.srv['set_analog_out'] =   self.create_service(srv.SetAnalogOut, 'set_analog_out', self.set_analog_output_callback)

        # Initialize serial port
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.2)
        self.get_logger().info('TriggerscopeServer node initialized')


    def set_digital_output_callback(self, request:srv.SetDigitalOut_Request, response:srv.SetDigitalOut_Response) -> srv.SetDigitalOut_Response:
        self.get_logger().debug('Received request to set digital output %d to %d' % (request.channel, request.state))
        outstring = f"SDO{int(request.channel)}-{int(request.state)}"
                
        response.success = self.send_command_read_reply(outstring)
        if response.success:
            self.get_logger().debug('Successfully set digital output %d to %d' % (request.channel, request.state))
        else:
            self.get_logger().error('Failed to set digital output %d to %d' % (request.channel, request.state))
        return response

    def set_analog_range_callback(self, request:srv.SetAnalogRange_Request, response:srv.SetAnalogRange_Response) -> srv.SetAnalogRange_Response:
        self.get_logger().debug('Received request to set analog range %d to %d' % (request.channel, request.range))
        outstring = f"SAR{int(request.channel)}-{int(request.range)}"

        response.success = self.send_command_read_reply(outstring)
        if response.success:
            # Store the analog range for future reference
            self.analog_ranges[request.channel] = self.ANALOG_VOLTAGE_RANGES[request.range]
            self.get_logger().debug('Successfully set analog range %d to %d' % (request.channel, request.range))
        else:
            self.get_logger().error('Failed to set analog range %d to %d' % (request.channel, request.range))

        return response

    def set_analog_output_callback(self, request:srv.SetAnalogOut_Request, response:srv.SetAnalogOut_Response) -> srv.SetAnalogOut_Response:

        self.get_logger().debug('Received request to set analog output %d to %d' % (request.channel, request.voltage))

        # Function to rescale a value from one range to another (linear interpolation)
        rescale = lambda x, in_min, in_max, out_min, out_max: (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        try:
            if(request.voltage < self.analog_ranges[request.channel][0] or request.voltage > self.analog_ranges[request.channel][1]):
                self.get_logger().error('Voltage out of range for channel %d' % request.channel)
                response.success = False
                return response
            
            # Convert the voltage to a digital value
            digital = math.floor(rescale(request.voltage, self.analog_ranges[request.channel][0], self.analog_ranges[request.channel][1], self.DAC_MIN, self.DAC_MAX+1))
            digital = min(digital, self.DAC_MAX) # Because the max value is NOT inclusive, this should only be necessary for the exact max value
        except KeyError:
            self.get_logger().error('Analog voltage range unknown for channel %d' % request.channel)
            response.success = False
            return response
        
        outstring = f"SAO{int(request.channel)}-{int(digital)}"

        response.success = self.send_command_read_reply(outstring)
        if response.success:
            self.get_logger().debug('Successfully set analog output %d to %d' % (request.channel, request.voltage))
        else:
            self.get_logger().error('Failed to set analog output %d to %d' % (request.channel, request.voltage))

        return response
    
    def send_command_read_reply(self, command:str) -> bool:
        self.serial_port.flushInput()
        self.serial_port.flushOutput()
        self.serial_port.write((command + '\n').encode("ascii"))
        reply = self.serial_port.readline().strip().decode('ascii')
        success = (reply is not None) and (reply == '!' + command)
        if(not success):
            self.get_logger().error('Triggerscope responded \'%s\'' % reply)
        return success
    
def main(args=None):

    rclpy.init(args=args)

    try:
        triggerscopeserver = TriggerscopeServerNode()
        rclpy.spin(triggerscopeserver)
    except ExternalShutdownException:
        triggerscopeserver.get_logger().error('External shutdown')
    except KeyboardInterrupt:
        triggerscopeserver.get_logger().error('Keyboard interrupt')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    finally:
        triggerscopeserver.destroy_node()
        
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
