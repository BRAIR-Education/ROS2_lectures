# My First Package in ROS2 (Python)
To start with ROS2, we're going to create our first ROS package from the official tutorial of [ROS Documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).

## 1) Create Package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --license Apache-2.0 py_pubsub
```

## 2) Create Publisher Node
Navigate in the `py_pubsub` directory.
```bash
cd py_pubsub
```
Create `publisher_member_function.py` file and copying this:
```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 3) Create Subscriber Node
Create `subscriber_member_function`:
```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 4) Modify Manifest File (`package.xml`)
```diff
<test_depend>python3-pytest</test_depend>

+ <!-- Dependencies -->
+ <exec_depend>rclpy</exec_depend>
+ <exec_depend>std_msgs</exec_depend>

<export>
<build_type>ament_python</build_type>
</export>
```

## 5) Modify Setup File (`setup.py`)
```diff
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
+            'talker = py_pubsub.publisher_member_function:main',
+            'listener = py_pubsub.subscriber_member_function:main',
        ],
    },
)
```

## 6) Build
```bash
cd ~/ros2_w
colcon build --packages-select py_pubsub
```

## 7) Run
Run the talker node:
```bash
ros2 run py_pubsub talker
```
Run the subscriber node in another terminal:
```bash
ros2 run py_pubsub listener
```

## 8) Launch Nodes
Launching the nodes in different terminals is annoying. For this reason, ROS provides you the `launch` files. The launch files are special files (written in `.xml`, `.yaml` or `.py`) that launches simultaneously the nodes. To do it in ROS2:

### 8.1) Create `.xml` file
Create a directory `launch`, typing
```bash
cd ~/ros2_ws/src/py_pubsub
mkdir launch
```
Create a launch file (`pubsub_launch.xml`) copying the following code
```xml
<launch>
    <node name="listener_node" pkg="py_pubsub" exec="listener" output="screen"/>
    <node name="talker_node" pkg="py_pubsub" exec="talker" output="screen"/>
</launch>
```

### 8.2) Import the launch files in the `setup.py`
```diff
        ('share/' + package_name, ['package.xml']),
+        # Include all launch files.
+        (os.path.join('share', package_name, 'launch'),
+        glob(os.path.join('launch', '*launch.[pxy][yma]*'))
+        ) # assumes _launch suffix
    ],
```

### 8.3) Launch Talker and Listener
```bash
ros2 launch py_pubsub pubsub_launch.xml
```
