import time

from mantis_msgs.action import PhaseAcquisition
from triggerscope_msgs.srv import SetAnalogSequence, SetAnalogOut, ControlAnalogSequence, ControlAnalogSequence_Request
from triggerscope_msgs.srv import SetDigitalOut
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class PhaseAcquisitionActionServer(Node):

    def __init__(self):
        super().__init__('phase_acquisition_action_server')

        self.volts_per_um = 3.0
        self.use_analog_sequence = True
        
        self.stage_analog_channel = self.declare_parameter("stage_analog_channel", 3).get_parameter_value().integer_value
        self.light_source_digital_channel = self.declare_parameter("light_source_digital_channel", 0).get_parameter_value().integer_value
        
        self.action_server = ActionServer(
            self,
            PhaseAcquisition,
            'run_phase_acquisition',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.stage_sequence_client = self.create_client(SetAnalogSequence, 'set_analog_sequence')
        self.stage_move_client = self.create_client(SetAnalogOut, 'set_analog_out')
        self.stage_sequence_control_client= self.create_client(ControlAnalogSequence, 'control_analog_sequence')
        self.light_source_client= self.create_client(SetDigitalOut, 'set_digital_out')

        self.get_logger().info('Phase Acquisition Action Server has been started')
        
    def destroy(self):
        self.action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
                
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute a goal."""
        self.get_logger().info('Executing goal...')


        self.stage_move_client.wait_for_service(timeout_sec=1.0)
        self.stage_sequence_client.wait_for_service(timeout_sec=1.0)
        self.stage_sequence_control_client.wait_for_service(timeout_sec=1.0)
        self.light_source_client.wait_for_service(timeout_sec=1.0)
        if not (self.stage_move_client.service_is_ready()):
            self.get_logger().error('Triggerscope services not available')
            goal_handle.abort()
            return PhaseAcquisition.Result()
        
        # Turn on the light source
        light_request = SetDigitalOut.Request(channel_bank=False, state=[False,]*8)
        light_request.state[self.light_source_digital_channel] = True
        light_result = self.light_source_client.call(light_request)
        if light_result.success:
            self.get_logger().info('Successfully turned on light source')
        else:
            self.get_logger().error('Failed to turn on light source')
            goal_handle.abort()
            return PhaseAcquisition.Result()
        
        if self.use_analog_sequence:
            
            # Clear the analog sequence
            control_request = ControlAnalogSequence.Request()
            control_request.channel = self.stage_analog_channel
            control_request.command = ControlAnalogSequence_Request.ANALOG_SEQUENCE_COMMAND_CLEAR
            control_result = self.stage_sequence_control_client.call(control_request)
            if control_result.success:
                self.get_logger().info('Successfully cleared analog sequence')
            else: 
                self.get_logger().error('Failed to clear analog sequence')
                goal_handle.abort()
                return PhaseAcquisition.Result()
            
            
            # Set the analog sequence
            request = SetAnalogSequence.Request()
            request.channel = self.stage_analog_channel
            request.voltages = [float(i) for i in range(0, 11)] 
            
            result = self.stage_sequence_client.call(request)
            if result.success:
                self.get_logger().info('Successfully set analog sequence')
            else:
                self.get_logger().error('Failed to set analog sequence')
                goal_handle.abort()
                return PhaseAcquisition.Result()
            
            
            # Start the analog sequence
            control_request.command = ControlAnalogSequence_Request.ANALOG_SEQUENCE_COMMAND_START
            control_result = self.stage_sequence_control_client.call(control_request)
            
            if control_result.success:
                self.get_logger().info('Successfully started analog sequence')
            else:
                self.get_logger().error('Failed to start analog sequence')
                goal_handle.abort()
                return PhaseAcquisition.Result()
            
        else:
            # Set the analog sequence
            request = SetAnalogOut.Request()        
            sequence = [float(i) for i in range(0, 11)]
            
            # Append the seeds for the PhaseAcquisition sequence
            feedback_msg = PhaseAcquisition.Feedback()

            for i in sequence:
                request.channel = self.stage_analog_channel
                request.voltage = i
                #await result = self.stage_move_client.call_async(request)
                result = self.stage_move_client.call(request)
                if result.success:
                    self.get_logger().info('Successfully set analog output to %d' % i)
                else:
                    self.get_logger().error('Failed to set analog output to %d' % i)
                    goal_handle.abort()
                    return PhaseAcquisition.Result()
                #feedback_msg.current_position_nm = i * self.volts_per_nm
                #goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.2)


        # Turn on the off source
        light_request.state[self.light_source_digital_channel] = False
        light_result = self.light_source_client.call(light_request)
        if light_result.success:
            self.get_logger().info('Successfully turned on light source')
        else:
            self.get_logger().error('Failed to turn on light source')
            goal_handle.abort()
            return PhaseAcquisition.Result()
        
        self.get_logger().info('Successfully completed goal')
        goal_handle.succeed()

        # Populate result message
        result = PhaseAcquisition.Result()

        return result


def main(args=None):
    rclpy.init(args=args)

    phase_acquisition_action_server = PhaseAcquisitionActionServer()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(phase_acquisition_action_server, executor=executor)

    phase_acquisition_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
