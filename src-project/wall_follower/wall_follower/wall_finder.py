from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from custom_interfaces.srv import FindWall
import rclpy
from rclpy.node import Node
import numpy as np
import time

class WallFinderServer(Node):
    def __init__(self):
        super().__init__('find_wall_server')
        self.real_robot = True
        self.group = ReentrantCallbackGroup()

        # Service
        self.srv = self.create_service(
            FindWall,
            'find_wall', 
            self.custom_service_callback,
            callback_group=self.group
        )
        # Subscriber
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self.group
        )
        # Publisher
        self.publisher_ = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.laser_scans = []
        self.laser_forward = 0
        self.find_wall_tolerance = 10
        self.cmd = Twist()
        self.init_conf_lidar = True

    def laser_callback(self, msg):
        self.laser_scans = []
        for i in range(0, 720):
            if msg.ranges[i] < 0.1 or msg.ranges[i] > 3.0:
                self.laser_scans.append(3.0)  # Filtering out invalid data
            else:
                self.laser_scans.append(msg.ranges[i])
        
        if self.real_robot:
            self.laser_forward = msg.ranges[360]
            self.right_id = 180
            self.front_id = 360
        else:
            self.laser_forward = msg.ranges[0]
            self.right_id = 540
            self.front_id = 0
        
        self.min_distance_index = self.laser_scans.index(min(self.laser_scans))

    def custom_service_callback(self, request, response):
        msg = Twist()

        self.get_logger().info(f'Total len: {len(self.laser_scans)}')
        min_distance = min(self.laser_scans)
        self.get_logger().info(f'Minimum Distance Index: {self.min_distance_index}')
        self.get_logger().info(f'Minimum Distance is: {min_distance}')

        # Turn to face the wall
        while abs(self.min_distance_index - self.front_id) > self.find_wall_tolerance:
            self.get_logger().info(f'Minimum Distance Index: {self.min_distance_index}')
            msg.angular.z = 0.25   
            self.publisher_.publish(msg)
            time.sleep(0.3)
            self.get_logger().info('Finding the wall...')

        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Wall found')

        # Move to the wall if it isn't close enough
        if min_distance > 0.3:
            linear_speed = 0.05
            distance_to_travel = min_distance - 0.3
            time_to_travel = distance_to_travel / linear_speed
            start_time = time.time()
            while time.time() - start_time <= time_to_travel:
                msg.linear.x = linear_speed
                self.publisher_.publish(msg)
            msg.linear.x = 0.0
            self.publisher_.publish(msg)
        
        # Rotate to have the wall on the right
        while abs(self.min_distance_index - self.right_id) > self.find_wall_tolerance:
            msg.angular.z = 0.25   
            self.publisher_.publish(msg)
            time.sleep(0.3)
            self.get_logger().info('Rotating to have the wall on the right...')

        msg.angular.z = 0.0
        self.publisher_.publish(msg)

        response.wallfound = True
    
        self.get_logger().info('Request - ready to send')
        return response


def main(args=None):
    rclpy.init(args=args)
    wall_finder_node = WallFinderServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(wall_finder_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        wall_finder_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
