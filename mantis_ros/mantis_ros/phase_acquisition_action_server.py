import time

from mantis_msgs.action import PhaseAcquisition
from triggerscope_msgs.srv import SetAnalogSequence, SetAnalogOut, ControlAnalogSequence, ControlAnalogSequence_Request
from triggerscope_msgs.srv import SetDigitalOut, SetAnalogRange, SetAnalogRange_Request

from flir_camera_msgs.action import AcquireMultiFrame
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus



class PhaseAcquisitionActionServer(Node):

    def __init__(self):
        super().__init__('phase_acquisition_action_server')

        self.volts_per_um = 30.0 ** -1
        
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

        self.analog_range_client = self.create_client(SetAnalogRange, 'set_analog_range')
        self.stage_sequence_client = self.create_client(SetAnalogSequence, 'set_analog_sequence')
        self.stage_move_client = self.create_client(SetAnalogOut, 'set_analog_out')
        self.stage_sequence_control_client= self.create_client(ControlAnalogSequence, 'control_analog_sequence')
        self.light_source_client= self.create_client(SetDigitalOut, 'set_digital_out')
        self.camera_action = ActionClient(self, AcquireMultiFrame, 'acquire_multi_frame')

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
        self.camera_action.wait_for_server(timeout_sec=1.0)
        
        if not (self.stage_move_client.service_is_ready()):
            self.get_logger().error('Triggerscope services not available')
            goal_handle.abort()
            return PhaseAcquisition.Result()
        
        set_range_request = SetAnalogRange.Request()
        set_range_request.channel = self.stage_analog_channel
        set_range_request.range = SetAnalogRange_Request.VOLTAGE_RANGE_0_TO_10
        set_range_result = self.analog_range_client.call(set_range_request)
        if set_range_result.success:
            self.get_logger().info('Successfully set analog range')
        else: 
            self.get_logger().error('Failed to set analog range')
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
        step_size = (goal_handle.request.end_pos_um-goal_handle.request.start_pos_um)/(goal_handle.request.num_frames-1)
        request.voltages = [float(goal_handle.request.start_pos_um + i*step_size)*self.volts_per_um for i in range(0, goal_handle.request.num_frames)]
                    
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
        
        self.camera_action.wait_for_server(timeout_sec=1.0)

        goal_msg = AcquireMultiFrame.Goal()
        goal_msg.num_frames = len(request.voltages)
        
        camera_response = self.camera_action.send_goal(goal_msg)
        if camera_response.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Successfully ran camera sequence')
        else:
            self.get_logger().error('Failed to run camera sequence')
            goal_handle.abort()
            return PhaseAcquisition.Result()


        # Stop the analog sequence
        control_request.command = ControlAnalogSequence_Request.ANALOG_SEQUENCE_COMMAND_STOP
        control_result = self.stage_sequence_control_client.call(control_request)
        
        if control_result.success:
            self.get_logger().info('Successfully stopped analog sequence')
        else:
            self.get_logger().error('Failed to stop analog sequence')
            goal_handle.abort()
            return PhaseAcquisition.Result()
        
        # Turn on the off source
        light_request.state[self.light_source_digital_channel] = False
        light_result = self.light_source_client.call(light_request)
        if light_result.success:
            self.get_logger().info('Successfully turned off light source')
        else:
            self.get_logger().error('Failed to turn off light source')
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
