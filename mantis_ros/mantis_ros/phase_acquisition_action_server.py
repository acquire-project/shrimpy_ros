import time

from mantis_msgs.action import PhaseAcquisition
from triggerscope_msgs.srv import SetAnalogSequence
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class PhaseAcquisitionActionServer(Node):

    def __init__(self):
        super().__init__('phase_acquisition_action_server')

        self.volts_per_nm = 0.1

        self.action_server = ActionServer(
            self,
            PhaseAcquisition,
            'run_phase_acquisition',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.stage_client = self.create_client(SetAnalogSequence, 'set_analog_sequence')
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


        self.stage_client.wait_for_service(timeout_sec=1.0)
        if not self.stage_client.service_is_ready():
            self.get_logger().error('Service not available')
            goal_handle.abort()
            return PhaseAcquisition.Result()
        
        spacing = goal_handle.request.spacing_nm

        # Set the analog sequence
        request = SetAnalogSequence.Request()        
        #request.sequence = [i * self.volts_per_nm for i in range(0, 1000, spacing)]
        
        # Append the seeds for the PhaseAcquisition sequence
        feedback_msg = PhaseAcquisition.Feedback()


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