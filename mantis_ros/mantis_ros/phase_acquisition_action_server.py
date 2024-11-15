import time

from mantis_ros.action import PhaseAcquisition

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class PhaseAcquisitionActionServer(Node):

    def __init__(self):
        super().__init__('phase_acquisition_action_server')

        self._action_server = ActionServer(
            self,
            PhaseAcquisition,
            'phase_acquisition_action_server',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self.phase_acquisition_action_server.destroy()
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

        # Append the seeds for the PhaseAcquisition sequence
        feedback_msg = PhaseAcquisition.Feedback()


        goal_handle.succeed()

        # Populate result message
        result = PhaseAcquisition.Result()
        result.sequence = feedback_msg.sequence

        self.get_logger().info('Returning result: {0}'.format(result.sequence))

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
