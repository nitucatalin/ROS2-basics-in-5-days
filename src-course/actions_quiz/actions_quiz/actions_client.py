import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from actions_quiz_msg.action import ComputeDistance

class DistanceActionClient(Node):
    def __init__(self):
        super().__init__('distance_action_client')
        self._action_client = ActionClient(self, ComputeDistance, '/distance_as')

    def send_goal(self, seconds):
        goal_msg = ComputeDistance.Goal()
        goal_msg.seconds = seconds

        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal to compute distance for {seconds} seconds...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # Acest callback primește feedback de la server
        self.get_logger().info(f'Current distance: {feedback_msg.feedback.current_dist}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: Total distance: {result.total_dist}, Status: {result.status}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = DistanceActionClient()
    action_client.send_goal(20)  # Trimite un goal pentru 20 de secunde
    rclpy.spin(action_client)  # Rămâne în spin pentru a asculta feedback-ul și rezultatul

if __name__ == '__main__':
    main()
