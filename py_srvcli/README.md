# ROS Services Tutorial (Python)
In the last [exercise](https://github.com/BRAIR-Education/ROS2_lectures/tree/main/py_pubsub), we explored the ROS topics. In this exercise, we're going to understand the ROS services.

## 1) Create Package
Navigate in your workspace (e.g., `~/ros2_ws/src`) path and type:
```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 py_srvcli --dependencies rclpy example_interfaces
```
The `example_interfaces` dependence is a demo package created by ROS that stores the service that we want to use: `AddTwoInts`. You can find [here](https://github.com/ros2/example_interfaces/blob/jazzy/srv/AddTwoInts.srv) the `.srv` file. I paste here for reading comodity:
```
int64 a
int64 b
---
int64 sum
```
In the definition of the service, you can find two fiels: the former related to the request (`int64 a`, `int64 b`) and the latter to the response (`int64 sum`).

## 2) Write Service Node
Navigate in your package:
```bash
cd py_srvcli/py_srvcli
```
create a file named `service_member_function` and paste:
```python
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 3) Create the Client Node
In the same path (`~/ros2_ws/src/py_srvcli/py_srvcli`), create a file named `client_member_function` and paste:
```python
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 4) Entry Points
Modify your `setup.py` adding the following lines to create the entry points:
```diff
entry_points={
    'console_scripts': [
+        'service = py_srvcli.service_member_function:main',
+        'client = py_srvcli.client_member_function:main',
    ],
},
```

## 5) Build
To assure that you have all the dependencies installed, you can use `rosdep` that you should install with ROS2.
```bash
rosdep install -i --from-path src --rosdistro jazzy -y
```
If you have different distro from jazzy you can put your distro name (e.g., `humble`) or typing `${ROS_DISTRO}`.

After that, we can build our package, switching as usual in the workspace dir (`~/ros2_ws`) and typing
```bash
colcon build --packages-select py_srvcli
``` 

## 6) Run Server and Client
Open a new terminal or source the workspace (`source ~/ros2_ws/install/setup.bash`) and write:
```bash
ros2 run py_srvcli service
```
After that, open a new terminal and write:
```bash
ros2 run py_srvcli client
```