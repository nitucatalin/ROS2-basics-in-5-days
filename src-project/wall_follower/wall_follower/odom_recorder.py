import time
import math

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from custom_interfaces.action import OdomRecord

from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rclpy.qos import ReliabilityPolicy, QoSProfile


class MyActionServer(Node):

    def __init__(self):
        super().__init__('quiz_action_server')
        self.action_callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer (
            self, 
            OdomRecord, 
            'record_odom', 
            self.execute_callback, 
            callback_group=self.action_callback_group
        )
        self.last_odom = Point32()

        self.publisher_ = self.create_publisher (
            Float32, 
            'total_distance', 
            10
        )
        self.odom_callback_group = self.action_callback_group
        self.odom_subscriber = self.create_subscription ( 
            Odometry, 
            '/odom', 
            self.odom_callback, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), 
            callback_group=self.odom_callback_group
        )  

    def odom_callback(self, msg):
        self.last_odom.x = msg.pose.pose.position.x
        self.last_odom.y = msg.pose.pose.position.y

    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')
        feedback_msg = OdomRecord.Feedback()
        self.first_odom = Point32()
        self.first_odom.x = self.last_odom.x
        self.first_odom.y = self.last_odom.y
        self.last_x = self.first_odom.x
        self.last_y = self.first_odom.y
        self.odom_record = []
        self.total_distance = 0

        back = False
        steps = 0

        while(back == False):
            time.sleep(1)
            new_point = Point32()
            new_point.x = self.last_odom.x
            new_point.y = self.last_odom.y

            distance = math.sqrt((self.last_odom.x - self.last_x) ** 2 + (self.last_odom.y - self.last_y) ** 2)

            self.last_x = new_point.x
            self.last_y = new_point.y

            self.total_distance += distance

            feedback_msg.current_total = self.total_distance
            goal_handle.publish_feedback(feedback_msg)

            distance_first_last = math.sqrt((self.last_odom.x - self.first_odom.x) ** 2 + (self.last_odom.y - self.first_odom.y) ** 2)

            self.get_logger().info('Distance from start "%s"' % str(distance_first_last))

            if distance_first_last <= 0.20 and steps > 30:
                back = True

            steps += 1

            self.odom_record.append(new_point)

        goal_handle.succeed()
        result = OdomRecord.Result()
        result.list_of_odoms = self.odom_record
        self.get_logger().info('Result: {0}'.format(result)) 

        return result


def main(args=None):
    rclpy.init(args=args)
    my_action_server = MyActionServer()
    executor = MultiThreadedExecutor(num_threads=2)
    rclpy.spin(my_action_server, executor=executor)
    my_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()