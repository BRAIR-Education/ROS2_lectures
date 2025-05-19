import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class CarSubscriber(Node):

    def __init__(self):
        super().__init__('car_subs')
        self.subscription = self.create_subscription(Twist, "cmd_vel", self.twist_callback, 10)
        self.subscription
    def twist_callback(self, msg):
        self.get_logger().info('I heard: "%x"' % msg.linear.x)


def main(args=None):
    rclpy.init(args=args)

    car_subs = CarSubscriber()

    rclpy.spin(car_subs)

    car_subs.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    
