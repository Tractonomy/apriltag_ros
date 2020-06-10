import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    apriltag_ros_prefix = get_package_share_directory("apriltag_ros")
    composable_node = ComposableNode(
        node_name='apriltag',
        package='apriltag_ros', node_plugin='AprilTagNode',
        remappings=[("image", "/camera/color/image_raw"), ("camera_info", "/camera/color/camera_info")],
        parameters=[os.path.join(
                    apriltag_ros_prefix,
                    "cfg",
                    "tags_36h11_all.yaml",
                    )])
    container = ComposableNodeContainer(
        node_name='tag_container',
        node_namespace='apriltag',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([container])
