"""Launch a server and a client in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='cpp_component_service',
                    plugin='composition::Server',
                    name='talker'),
                ComposableNode(
                    package='cpp_component_service',
                    plugin='composition::Client',
                    name='listener')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
