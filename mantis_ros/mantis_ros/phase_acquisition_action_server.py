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

        try:
            self.stage_move_client.wait_for_service(timeout_sec=1.0)
            self.stage_sequence_client.wait_for_service(timeout_sec=1.0)
            self.stage_sequence_control_client.wait_for_service(timeout_sec=1.0)
            self.light_source_client.wait_for_service(timeout_sec=1.0)
            self.camera_action.wait_for_server(timeout_sec=1.0)
            
            if not (self.stage_move_client.service_is_ready()):
                self.get_logger().error('Triggerscope services not available')
                raise RuntimeError
            
            # Set the analog range
            set_range_request = SetAnalogRange.Request(channel=self.stage_analog_channel, range=SetAnalogRange_Request.VOLTAGE_RANGE_0_TO_10)
            self.call_triggerscope(self.analog_range_client, set_range_request, 'set analog range')
            
            # Turn on the light source
            light_request = SetDigitalOut.Request(channel_bank=False, state=[False,]*8)
            light_request.state[self.light_source_digital_channel] = True
            self.call_triggerscope(self.light_source_client, light_request, 'turn on light source')
                        
            # Clear the analog sequence
            control_request = ControlAnalogSequence.Request(channel=self.stage_analog_channel, command=ControlAnalogSequence_Request.ANALOG_SEQUENCE_COMMAND_CLEAR)
            self.call_triggerscope(self.stage_sequence_control_client, control_request, 'clear analog sequence')
            
            # Set the analog sequence
            step_size = (goal_handle.request.end_pos_um-goal_handle.request.start_pos_um)/(goal_handle.request.num_frames-1)
            voltages = [float(goal_handle.request.start_pos_um + i*step_size)*self.volts_per_um for i in range(0, goal_handle.request.num_frames)]
            request = SetAnalogSequence.Request(channel=self.stage_analog_channel, voltages=voltages)
            self.call_triggerscope(self.stage_sequence_client, request, 'set analog sequence')
            
            # Start the analog sequence
            control_request.command = ControlAnalogSequence_Request.ANALOG_SEQUENCE_COMMAND_START
            self.call_triggerscope(self.stage_sequence_control_client, control_request, 'start analog sequence')
            
            
            # Call the AcquireMultiFrame action on the camera
            self.camera_action.wait_for_server(timeout_sec=1.0)
            goal_msg = AcquireMultiFrame.Goal()
            goal_msg.num_frames = len(request.voltages)
            
            camera_response = self.camera_action.send_goal(goal_msg)
            if camera_response.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Successfully ran camera sequence')
            else:
                self.get_logger().error('Failed to run camera sequence')
                raise RuntimeError
            
            # Turn on the off source
            light_request.state[self.light_source_digital_channel] = False
            self.call_triggerscope(self.light_source_client, light_request, 'turn off light source')
            
            # Stop the analog sequence
            control_request.command = ControlAnalogSequence_Request.ANALOG_SEQUENCE_COMMAND_STOP
            self.call_triggerscope(self.stage_sequence_control_client, control_request, 'stop analog sequence')
            
        except RuntimeError:
            self.get_logger().error('Aborting goal')
            goal_handle.abort()
        else:
            self.get_logger().info('Successfully completed goal')
            goal_handle.succeed()

        # Populate result message
        result = PhaseAcquisition.Result()

        return result
    
    def call_triggerscope(self, server, request, logstring):
        """Call a service on the Triggerscope."""
            
        result = server.call(request)
        if result.success:
            self.get_logger().info('Succeeded: ' + logstring)
        else:
            self.get_logger().error('Failed: ' + logstring)
            raise RuntimeError
            
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
