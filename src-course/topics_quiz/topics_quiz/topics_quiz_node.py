import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class TopicsQuizNode(Node):

    def __init__(self):
        super().__init__('topics_quiz_node')

        # Publisher pentru /cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber pentru /odom și /scan (LIDAR)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # Inițializează variabile
        self.move_cmd = Twist()
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0
        self.state = 'move_forward'
        self.target_x_forward = 2.0  # Distanța până la care se mișcă în față
        self.target_yaw_turn = math.radians(90)  # 90 de grade rotire
        self.has_rotated = False
        self.laser_data = []

        self.timer = self.create_timer(0.1, self.move_robot)

    def odom_callback(self, msg):
        # Actualizează poziția și orientarea
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convertește quaternion în unghiul yaw
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def laser_callback(self, msg):
        # Salvăm datele LIDAR pentru a le folosi ulterior
        self.laser_data = msg.ranges

    def move_robot(self):
        if self.state == 'move_forward':
            # Verifică dacă există obstacole în față
            if len(self.laser_data) > 0:
                front_laser_data = list(self.laser_data[0:30]) + list(self.laser_data[330:359])
                min_distance_in_front = min(front_laser_data)

                # Dacă obstacolul din față e prea aproape, oprește robotul
                if min_distance_in_front < 0.5:
                    self.get_logger().info("Obstacle detected! Stopping and rotating...")
                    self.state = 'rotate'

            # Mișcă robotul înainte până când ajunge la poziția dorită
            if self.current_x < self.target_x_forward:
                self.move_cmd.linear.x = 0.2
                self.move_cmd.angular.z = 0.0
            else:
                # Trecem la starea de rotire dacă am ajuns la poziția dorită
                self.state = 'rotate'

        elif self.state == 'rotate':
            # Rotire la 90 de grade
            if not self.has_rotated:
                self.rotate_90_degrees()
        elif self.state == 'move_through_gap':
            # Mișcă robotul înainte prin deschizătură după rotire
            self.move_cmd.linear.x = 0.2
            self.move_cmd.angular.z = 0.0

        # Publică comanda de mișcare
        self.publisher_.publish(self.move_cmd)

    def rotate_90_degrees(self):
        # Dacă diferența dintre yaw-ul curent și ținta de 90 de grade este suficient de mică
        if abs(self.yaw - self.target_yaw_turn) > 0.05:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.3  # Rotire la dreapta
        else:
            # Dacă a terminat rotirea, trecem la starea de mișcare prin deschizătură
            self.move_cmd.angular.z = 0.0
            self.state = 'move_through_gap'
            self.has_rotated = True

def main(args=None):
    rclpy.init(args=args)
    topics_quiz_node = TopicsQuizNode()
    rclpy.spin(topics_quiz_node)
    topics_quiz_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
