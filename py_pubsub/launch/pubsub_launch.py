from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Equivalent of
        # <node name="listener_node" pkg="py_pubsub" exec="listener" output="screen"/>
        Node(
            package='py_pubsub',
            executable='listener',
            name='listener_node',
            output='screen'
        ),
        # Equivalent of
        # <node name="talker_node" pkg="py_pubsub" exec="talker" output="screen"/>
        Node(
            package='py_pubsub',
            executable='talker',
            name='talker_node',
            output='screen'
        )
    ])