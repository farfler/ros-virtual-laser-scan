from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    launch_virtual_point_cloud_node = Node(
        package="virtual_laser_scan",
        executable="virtual_point_cloud",
        name="virtual_point_cloud",
    )

    launch_virtual_laser_scan_node = Node(
        package="virtual_laser_scan",
        executable="virtual_laser_scan",
        name="virtual_laser_scan",
    )

    return LaunchDescription(
        [
            launch_virtual_point_cloud_node,
            launch_virtual_laser_scan_node,
        ]
    )
