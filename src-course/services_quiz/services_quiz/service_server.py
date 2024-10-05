from geometry_msgs.msg import Twist
from services_quiz_srv.srv import Turn
import rclpy
from rclpy.node import Node
import time

class Service(Node):

    def __init__(self):
        super().__init__('quiz_server')

        self.srv = self.create_service(Turn, 'turn', self.custom_service_callback)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        

    def custom_service_callback(self, request, response):
        msg = Twist()

        response.success = True
        spin_direction = 0
        
        if request.direction == "right":
            spin_direction = -1
            self.get_logger().info('Turning to right direction!!')
        elif request.direction == "left":
            spin_direction = 1
            self.get_logger().info('Turning to left direction!!')
        else:
            # response state
            response.success = False

        speed = request.angular_velocity * spin_direction
    
        msg.angular.z = speed
        self.publisher_.publish(msg)

        time.sleep(request.time)

        msg.angular.z = 0.0
        self.publisher_.publish(msg)

        return response


def main(args=None):
    rclpy.init(args=args)
    service = Service()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()