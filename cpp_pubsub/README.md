# My First Package in ROS2 (C++)
To start with ROS2, we're going to create our first ROS package from the official tutorial of [ROS Documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html).

## 1) Create Package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_pubsub
```

## 2) Create Publisher Node
Navigate in the `cpp_pubsub` directory.
```bash
cd cpp_pubsub
```
Create `publisher_lambda_function.cpp` file and copying this:
```cpp
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
* function to shorten the callback syntax, at the expense of making the
* code somewhat more difficult to understand at first glance. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

## 3) Create Subscriber Node
Create `subscriber_lambda_function`:
```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    auto topic_callback =
      [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
    subscription_ =
      this->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

## 4) Modify Manifest File (`package.xml`)
```diff
<buildtool_depend>ament_cmake</buildtool_depend>

+ <!-- Dependencies -->
+ <depend>rclpy</depend>
+ <depend>std_msgs</depend>

<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>

<export>
<build_type>ament_cmake</build_type>
</export>
```

## 5) Modify `CMakeLists.txt`
```diff
cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
+find_package(rclcpp REQUIRED)
+find_package(std_msgs REQUIRED)

+add_executable(talker src/publisher_lambda_function.cpp)
+9ament_target_dependencies(talker rclcpp std_msgs)
a+dd_executable(listener src/subscriber_lambda_function.cpp)
+ament_target_dependencies(listener rclcpp std_msgs)

+install(TARGETS
+  talker
+  listener
+  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

## 6) Build
```bash
cd ~/ros2_w
colcon build --packages-select cpp_pubsub
```

## 7) Run
Run the talker node:
```bash
ros2 run cpp_pubsub talker
```
Run the subscriber node in another terminal:
```bash
ros2 run cpp_pubsub listener
```

## 8) Launch Nodes
Launching the nodes in different terminals is annoying. For this reason, ROS provides you the `launch` files. The launch files are special files (written in `.xml`, `.yaml` or `.py`) that launches simultaneously the nodes. To do it in ROS2:

### 8.1) Create `.xml` file
Create a directory `launch`, typing
```bash
cd ~/ros2_ws/src/cpp_pubsub
mkdir launch
```
Create a launch file (`pubsub_launch.xml`) copying the following code
```xml
<launch>
    <node name="listener_node" pkg="cpp_pubsub" exec="listener" output="screen"/>
    <node name="talker_node" pkg="cpp_pubsub" exec="talker" output="screen"/>
</launch>
```

### 8.2) Import the launch files in the `CMakeLists.txt`
```diff
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
+install(DIRECTORY
+  launch
+  DESTINATION share/${PROJECT_NAME}/
+)
```

### 8.3) Launch Talker and Listener
```bash
ros2 launch cpp_pubsub pubsub_launch.xml
```
