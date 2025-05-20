# Kinematic Unicycle Exercise (Python)
In this exercise, we will trying to simulate the Kinematic Model of Unicycle in a ROS2 node. 

## 1) Kinematic Model
The Kinematic Model of Unicycle can be written as:

$$ 
\dot{q}(t) = S(q) \ \nu(t) \rightarrow 
\begin{pmatrix} 
\dot{x} \\
\dot{y} \\
\dot{\theta} \end{pmatrix} = \begin{pmatrix} 
                                \cos(\theta) & 0 \\
                                \sin(\theta) & 0 \\
                                0 & 1 
                            \end{pmatrix}
                                        \begin{pmatrix} 
                                            v \\
                                            \omega
                                        \end{pmatrix}
$$

In order to implement the equation in our node, we have to discretize this differential equation. Using Forward Euler, we can write:
$$\dot{q} \approx \frac{q(k+1) - q(k)}{T_s}$$

where $T_s$ is the sampling period. Finally, we can discretize the Kinematic Model of Unicycle:

$$q(k + 1) = q(k) + T_s \ S(q) \ \nu(k)$$

Expanding the previous equation:

$$\begin{cases}
x_{k+1} = x_k + T_s \ \cos(\theta_k) v_k \\
y_{k+1} = y_k + T_s \ \sin(\theta_k) v_k \\
\theta_{k+1} = \theta_{k} + T_s \ \omega_k
\end{cases}$$

The theoretical part is over, let's start to code.

## 2) Organize the code
The node that we want to write, has as input $\nu = [v \quad \omega]^T$ and output $q = [x \quad y \quad \theta]^T$. We have to choice the ROS msgs for this node. A choice can be:
- Input: [geometry_msgs/Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)
- Output: [geometry_msgs/Pose2D](https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose2D.html)

The messages definition can be found on the previous links.

## 3) Create Pkg
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --license Apache-2.0 kin_unicycle --dependencies rclpy std_msgs geometry_msgs
```

## 4) Create Simulation Node
```bash
cd kin_unicycle
```
Create the `kinematic_model.py` pasting the following:
```python
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
```

## 5) Add entry points in `setup.py`
```diff
    entry_points={
        'console_scripts': [
+            unicycle = kin_unicycle.kinematic_model:main'
        ],
    },
```
## 6) Build
```bash
colcon build --packages-select kin_unicycle
```

## 7) Run the Simulation Node
```bash
ros2 run kin_unicycle unicycle
```

## 8) Visualization with PlotJuggler
To visualize the state of the simulation, we can use [PlotJuggler](https://plotjuggler.io/).

### 8.1) Install PlotJuggler
```bash
sudo apt install ros-$ROS_DISTRO-plotjuggler-ros
```

### 8.2) Run PlotJuggler
```bash
ros2 run plotjuggler plotjuggler
```

## 9) Publish `Twist` msg by `rqt`
```bash
rqt
```
Navigate in the Topic plugins and press to Topic publisher. You can use easily the GUI to publish the `Twist` msg and command your Kinematic Unicycle.