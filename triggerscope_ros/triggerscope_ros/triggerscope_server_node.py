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
        self.srv['set_analog_sequence'] =   self.create_service(srv.SetAnalogSequence, 'set_analog_sequence', self.set_analog_sequence_callback)
        self.srv['control_analog_sequence'] =   self.create_service(srv.ControlAnalogSequence, 'control_analog_sequence', self.control_analog_sequence_callback)

        # Initialize serial port
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.2)
        self.get_logger().info('TriggerscopeServer node initialized')


    def set_digital_output_callback(self, request:srv.SetDigitalOut_Request, response:srv.SetDigitalOut_Response) -> srv.SetDigitalOut_Response:

        
        self.get_logger().info(f'Received request to set digital output {request.channel_bank} to {request.state}')
        
        state_bits = 0
        for i in range(8):
             state_bits += request.state[i] << i

        outstring = f"SDO{int(request.channel_bank)}-{state_bits}"
        
        response.success = self.send_command_check_response(outstring)
        if response.success:
            self.get_logger().info(f'Successfully set digital output {request.channel_bank} to {request.state}')
        else:
            self.get_logger().error(f'Failed to set digital output {request.channel_bank} to {request.state}')
        return response

    def set_analog_range_callback(self, request:srv.SetAnalogRange_Request, response:srv.SetAnalogRange_Response) -> srv.SetAnalogRange_Response:
        self.get_logger().info('Received request to set analog range %d to %d' % (request.channel, request.range))
        outstring = f"SAR{int(request.channel)}-{int(request.range)}"

        response.success = self.send_command_check_response(outstring)
        if response.success:
            # Store the analog range for future reference
            self.analog_ranges[request.channel] = self.ANALOG_VOLTAGE_RANGES[request.range]
            self.get_logger().info('Successfully set analog range %d to %d' % (request.channel, request.range))
        else:
            self.get_logger().error('Failed to set analog range %d to %d' % (request.channel, request.range))

        return response

    def set_analog_output_callback(self, request:srv.SetAnalogOut_Request, response:srv.SetAnalogOut_Response) -> srv.SetAnalogOut_Response:

        self.get_logger().info('Received request to set analog output %d to %d' % (request.channel, request.voltage))

        self.success = False
        try:
            # Convert the voltage to a digital value
            digital = self.volts_to_digital(request.voltage, request.channel)
        except KeyError:
            self.get_logger().error('Analog voltage range unknown for channel %d' % request.channel)
            response.success = False
        except ValueError:
            self.get_logger().error('Voltage out of range for channel %d' % request.channel)
        else:
            # if none of the exceptions were raised, then we can proceed
            outstring = f"SAO{int(request.channel)}-{int(digital)}"
            response.success = self.send_command_check_response(outstring)
            
        if response.success:
            self.get_logger().info('Successfully set analog output %d to %d' % (request.channel, request.voltage))
        else:
            self.get_logger().error('Failed to set analog output %d to %d' % (request.channel, request.voltage))

        return response
    
    def set_analog_sequence_callback(self, request:srv.SetAnalogSequence_Request, response:srv.SetAnalogSequence_Response) -> srv.SetAnalogSequence_Response:
        self.get_logger().info('Received request to set analog sequence')
        response.success = False
        try:
            outstring = f"PAO{request.channel}-0-{'-'.join([str(self.volts_to_digital(voltage, request.channel)) for voltage in request.voltages])}"
        except KeyError:
            self.get_logger().error('Analog voltage range unknown for channel %d' % request.channel)
        except ValueError:
            self.get_logger().error('Voltage out of range for channel %d' % request.channel)
        else:
            # if none of the exceptions were raised, then we can proceed
            reply = self.send_command_read_reply(outstring)
            response.success = reply.startswith(f"!PAO{request.channel}-0-{len(request.voltages)}")
            
        if response.success:
            self.get_logger().info('Successfully set analog sequence')
        else:
            self.get_logger().error('Failed to set analog sequence')
        return response

    def control_analog_sequence_callback(self, request:srv.ControlAnalogSequence_Request, response:srv.ControlAnalogSequence_Response) -> srv.ControlAnalogSequence_Response:
        self.get_logger().info('Received request to control analog sequence')
        
        if request.command == srv.ControlAnalogSequence_Request.ANALOG_SEQUENCE_COMMAND_CLEAR:
            outstring = f"PAC{int(request.channel)}"
        else:
            outstring = f"PAS{int(request.channel)}-{int(request.command)}-{int(request.edge)}"

        response.success = self.send_command_check_response(outstring)
        if response.success:
            self.get_logger().info('Successfully controlled analog sequence')
        else:
            self.get_logger().error('Failed to control analog sequence')
        return response

    def send_command_read_reply(self, command:str) -> str:
        self.serial_port.flushInput()
        self.serial_port.flushOutput()
        
        self.get_logger().debug('SerialTX: \'%s\'' % command)
        self.serial_port.write((command + '\n').encode("ascii"))
        
        reply = self.serial_port.readline().strip().decode('ascii')
        self.get_logger().debug('SerialRX: \'%s\'' % reply)
        return reply
    
    def send_command_check_response(self, command:str) -> bool:
        reply = self.send_command_read_reply(command)
        return reply == '!' + command
    
    def volts_to_digital(self, volts:float, channel:int) -> int:
        """
        Convert a voltage to a digital value for a given channel
        """
        
        # the following checks if the voltage is within the range for the channel
        # a side effect is that it will raise a KeyError if the channel is not in the analog_ranges dictionary
        if(volts < self.analog_ranges[channel][0] or volts > self.analog_ranges[channel][1]):
            raise ValueError('Voltage out of range for channel %d' % channel)
        
        min_voltage, max_voltage = self.analog_ranges[channel]
        return min(math.floor((volts - min_voltage) / (max_voltage - min_voltage) * self.DAC_MAX+1), self.DAC_MAX)
    
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
