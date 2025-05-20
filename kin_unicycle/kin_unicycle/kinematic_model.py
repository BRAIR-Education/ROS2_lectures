# Import ROS Communication Library
import rclpy
from rclpy.node import Node

"""
The Node will publish geometry_msgs/Pose2D (state) and will subscribe to
geometry_msgs/Twist for the velocity command.
"""

# Import Messages
# Pose2D for the State
from geometry_msgs.msg import Pose2D
# Twist for the input
from geometry_msgs.msg import Twist

# import math for model
import numpy as np

## Useful Variables (sorry for the C-style programming)
QUEUE_SIZE = 10             # [-]
SAMPLING_FREQUENCY = 1000   # [Hz]

class Kinematic_Unicycle(Node):

    def __init__(self):
        ## Init Node
        super().__init__('kinematic_unicycle')

        # Publisher and Subscriber
        self.publisher_ = self.create_publisher(Pose2D, '/robot_state', QUEUE_SIZE)
        self.subscriber_ = self.create_subscription(Twist, '/cmd_vel', self.twist_callBack, QUEUE_SIZE)

        # avoid warning
        self.subscriber_

        ## Class Elements for simulation
        # State
        self.x = 0
        self.y = 0
        self.theta = 0

        # Input
        self.v = 0
        self.w = 0

        # Timer
        self.Ts = 1/SAMPLING_FREQUENCY # [s]
        self.timer_ = self.create_timer(self.Ts, self.simulation_loop)

    ### Callbacks ###
    def twist_callBack(self, msg):
        """
        v: x-coordinate of linear part of the msg.
        omega: z-coordinate of angular part of the msg.

        Note: physically the omega is correct, the v is only a magnitude information,
        since the components of the linear velocity are determined by the angle theta.
        """
        # Update control input
        self.v = msg.linear.x
        self.w = msg.angular.z

    def simulation_loop(self):
        """
        In the simulation loop (or main loop), you update
        your variables with the discrete differential equation.
        In the case of unicycle:
        x(k + 1) = x(k) + Ts*cos(theta)*v
        y(k + 1) = y(k) + Ts*sin(theta)*v
        theta(k + 1) = theta(k) + Ts*omega

        We used Forward Euler as discretization method.
        """
        # Update law of the variables
        self.x += self.Ts*np.cos(self.theta)*self.v
        self.y += self.Ts*np.sin(self.theta)*self.v
        self.theta += self.Ts*self.w

        # This avoid theta is out from [-pi, pi] range
        self.theta = self.__wrap2pi(self.theta)

        # Publish the updated state
        self.publish_state()

        """
        Note on the efficiency of this solution:
        The best should update directly the msg type Pose2D and
        avoiding to create and destroy the Pose2D just for publish.
        For huge simulation, this can be critical.
        """
    
    def publish_state(self):
        """
        This is a suboptimal solution to publish the message
        for learning purposes.
        """

        # Create msg object
        msg = Pose2D()

        # Fill
        msg.x = self.x
        msg.y = self.y
        msg.theta = self.theta

        # Publish
        self.publisher_.publish(msg)
    
    @staticmethod
    def __wrap2pi(angle):
        return np.arctan2(np.sin(angle), np.cos(angle))


def main(args=None):
    rclpy.init(args=args)

    kinematic_unicycle = Kinematic_Unicycle()

    rclpy.spin(kinematic_unicycle)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kinematic_unicycle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()