import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    apriltag_ros_prefix = get_package_share_directory("apriltag_ros")
    composable_node = ComposableNode(
        name='apriltag',
        package='apriltag_ros',
        plugin='AprilTagNode',
        remappings=[
            ("image", "/stereo_camera/color/image_raw"),
            ("camera_info", "/stereo_camera/color/camera_info")],
        parameters=[os.path.join(
                    apriltag_ros_prefix,
                    "cfg",
                    "tags_16h5_all.yaml",
                    )])
    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([container])
