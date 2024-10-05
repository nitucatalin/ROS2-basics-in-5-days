import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
# Import custom interface Age
from custom_interfaces.msg import Age


class Example36(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('example36')
        # create the publisher object
        self.publisher_ = self.create_publisher(Age, 'age', 10)
        # create an Age message
        self.age = Age()
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        # create an Age message
        self.age.year = 2031
        self.age.month = 5
        self.age.day = 21
        # publish the Age message
        self.publisher_.publish(self.age)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    example36 = Example36()
    rclpy.spin(example36)
    # Explicity destroy the node
    example36.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()