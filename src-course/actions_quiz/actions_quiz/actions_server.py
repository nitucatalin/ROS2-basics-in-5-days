import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from actions_quiz_msg.action import ComputeDistance
from geometry_msgs.msg import Twist
import time

class DistanceActionServer(Node):
    def __init__(self):
        super().__init__('distance_action_server')
        self._action_server = ActionServer(
            self,
            ComputeDistance,
            '/distance_as',
            self.execute_callback
        )
        self.publisher_ = self.create_publisher(Twist, '/total_distance', 10)
        self._total_distance = 0.0
        self._current_distance = 0.0

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal request: {goal_handle.request.seconds} seconds')
        seconds = goal_handle.request.seconds

        feedback_msg = ComputeDistance.Feedback()
        result_msg = ComputeDistance.Result()
        start_time = time.time()

        while (time.time() - start_time) < seconds:
            distance_increment = 0.1  # Simulare distanță parcursă pe secundă
            self._current_distance += distance_increment
            self._total_distance += distance_increment
            
            feedback_msg.current_dist = self._current_distance
            self.get_logger().info(f'Current distance: {self._current_distance}')

            # Publicare feedback și distanță totală
            goal_handle.publish_feedback(feedback_msg)

            msg = Twist()
            msg.linear.x = self._current_distance
            self.publisher_.publish(msg)
            
            time.sleep(1)

        result_msg.status = True
        result_msg.total_dist = self._total_distance
        goal_handle.succeed()

        self.get_logger().info(f'Action completed. Total distance: {self._total_distance}')
        return result_msg

def main(args=None):
    rclpy.init(args=args)
    action_server = DistanceActionServer()
    rclpy.spin(action_server)

if __name__ == '__main__':
    main()
