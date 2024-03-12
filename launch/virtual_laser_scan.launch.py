from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    front_left_laser_scan_topic = LaunchConfiguration(
        "front_left_laser_scan_topic", default="front_left_laser/scan"
    )

    front_right_laser_scan_topic = LaunchConfiguration(
        "front_right_laser_scan_topic", default="front_right_laser/scan"
    )

    rear_left_laser_scan_topic = LaunchConfiguration(
        "rear_left_laser_scan_topic", default="rear_left_laser/scan"
    )

    rear_right_laser_scan_topic = LaunchConfiguration(
        "rear_right_laser_scan_topic", default="rear_right_laser/scan"
    )

    virtual_laser_scan_topic = LaunchConfiguration(
        "virtual_laser_scan_topic", default="virtual_laser/scan"
    )

    virtual_laser_scan_frame_id = LaunchConfiguration(
        "virtual_laser_scan_frame_id", default="virtual_laser_frame"
    )

    virtual_laser_scan_angle_min = LaunchConfiguration(
        "virtual_laser_scan_angle_min", default="-3.14159265359"
    )

    virtual_laser_scan_angle_max = LaunchConfiguration(
        "virtual_laser_scan_angle_max", default="3.14159265359"
    )

    virtual_laser_scan_angle_increment = LaunchConfiguration(
        "virtual_laser_scan_angle_increment", default="0.01745329251"
    )

    virtual_laser_scan_time_increment = LaunchConfiguration(
        "virtual_laser_scan_time_increment", default="0.0"
    )

    virtual_laser_scan_scan_time = LaunchConfiguration(
        "virtual_laser_scan_scan_time", default="0.0"
    )

    virtual_laser_scan_range_min = LaunchConfiguration(
        "virtual_laser_scan_range_min", default="1.0"
    )

    virtual_laser_scan_range_max = LaunchConfiguration(
        "virtual_laser_scan_range_max", default="20.0"
    )

    virtual_point_cloud_topic = LaunchConfiguration(
        "virtual_point_cloud_topic", default="virtual_laser/cloud"
    )
    virtual_point_cloud_frame_id = LaunchConfiguration(
        "virtual_point_cloud_frame_id", default="virtual_laser_frame"
    )

    declare_frint_left_laser_scan_topic_launch_argument = DeclareLaunchArgument(
        "front_left_laser_scan_topic",
        default_value="front_left_laser/scan",
        description="TODO: Description of the front_left_laser_scan_topic argument.",
    )

    declare_front_right_laser_scan_topic_launch_argument = DeclareLaunchArgument(
        "front_right_laser_scan_topic",
        default_value="front_right_laser/scan",
        description="TODO: Description of the front_right_laser_scan_topic argument.",
    )

    declare_rear_left_laser_scan_topic_launch_argument = DeclareLaunchArgument(
        "rear_left_laser_scan_topic",
        default_value="rear_left_laser/scan",
        description="TODO: Description of the rear_left_laser_scan_topic argument.",
    )

    declare_rear_right_laser_scan_topic_launch_argument = DeclareLaunchArgument(
        "rear_right_laser_scan_topic",
        default_value="rear_right_laser/scan",
        description="TODO: Description of the rear_right_laser_scan_topic argument.",
    )

    declare_virtual_laser_scan_topic_launch_argument = DeclareLaunchArgument(
        "virtual_laser_scan_topic",
        default_value="virtual_laser/scan",
        description="TODO: Description of the virtual_laser_scan_topic argument.",
    )

    declare_virtual_laser_scan_frame_id_launch_argument = DeclareLaunchArgument(
        "virtual_laser_scan_frame_id",
        default_value="virtual_laser_frame",
        description="TODO: Description of the virtual_laser_scan_frame_id argument.",
    )

    declare_virtual_laser_scan_angle_min_launch_argument = DeclareLaunchArgument(
        "virtual_laser_scan_angle_min",
        default_value="-3.14159265359",
        description="TODO: Description of the virtual_laser_scan_angle_min argument.",
    )

    declare_virtual_laser_scan_angle_max_launch_argument = DeclareLaunchArgument(
        "virtual_laser_scan_angle_max",
        default_value="3.14159265359",
        description="TODO: Description of the virtual_laser_scan_angle_max argument.",
    )

    declare_virtual_laser_scan_angle_increment_launch_argument = DeclareLaunchArgument(
        "virtual_laser_scan_angle_increment",
        default_value="0.01745329251",
        description="TODO: Description of the virtual_laser_scan_angle_increment argument.",
    )

    declare_virtual_laser_scan_time_increment_launch_argument = DeclareLaunchArgument(
        "virtual_laser_scan_time_increment",
        default_value="0.0",
        description="TODO: Description of the virtual_laser_scan_time_increment argument.",
    )

    declare_virtual_laser_scan_scan_time_launch_argument = DeclareLaunchArgument(
        "virtual_laser_scan_scan_time",
        default_value="0.0",
        description="TODO: Description of the virtual_laser_scan_scan_time argument.",
    )

    declare_virtual_laser_scan_range_min_launch_argument = DeclareLaunchArgument(
        "virtual_laser_scan_range_min",
        default_value="1.0",
        description="TODO: Description of the virtual_laser_scan_range_min argument.",
    )

    declare_virtual_laser_scan_range_max_launch_argument = DeclareLaunchArgument(
        "virtual_laser_scan_range_max",
        default_value="20.0",
        description="TODO: Description of the virtual_laser_scan_range_max argument.",
    )

    declare_virtual_point_cloud_topic_launch_argument = DeclareLaunchArgument(
        "virtual_point_cloud_topic",
        default_value="virtual_laser/cloud",
        description="TODO: Description of the virtual_point_cloud_topic argument.",
    )

    declare_virtual_point_cloud_frame_id_launch_argument = DeclareLaunchArgument(
        "virtual_point_cloud_frame_id",
        default_value="virtual_laser_frame",
        description="TODO: Description of the virtual_point_cloud_frame_id argument.",
    )

    launch_virtual_point_cloud_node = Node(
        package="virtual_laser_scan",
        executable="virtual_point_cloud",
        name="virtual_point_cloud",
        parameters=[
            {
                "front_left_laser_scan_topic": front_left_laser_scan_topic,
                "front_right_laser_scan_topic": front_right_laser_scan_topic,
                "rear_left_laser_scan_topic": rear_left_laser_scan_topic,
                "rear_right_laser_scan_topic": rear_right_laser_scan_topic,
                "virtual_laser_scan_topic": virtual_laser_scan_topic,
                "virtual_laser_scan_frame_id": virtual_laser_scan_frame_id,
            }
        ],
    )

    launch_virtual_laser_scan_node = Node(
        package="virtual_laser_scan",
        executable="virtual_laser_scan",
        name="virtual_laser_scan",
        parameters=[
            {
                "virtual_laser_scan_topic": virtual_laser_scan_topic,
                "virtual_laser_scan_frame_id": virtual_laser_scan_frame_id,
                "virtual_point_cloud_topic": virtual_point_cloud_topic,
                "virtual_laser_scan_angle_min": virtual_laser_scan_angle_min,
                "virtual_laser_scan_angle_max": virtual_laser_scan_angle_max,
                "virtual_laser_scan_angle_increment": virtual_laser_scan_angle_increment,
                "virtual_laser_scan_time_increment": virtual_laser_scan_time_increment,
                "virtual_laser_scan_scan_time": virtual_laser_scan_scan_time,
                "virtual_laser_scan_range_min": virtual_laser_scan_range_min,
                "virtual_laser_scan_range_max": virtual_laser_scan_range_max,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_frint_left_laser_scan_topic_launch_argument,
            declare_front_right_laser_scan_topic_launch_argument,
            declare_rear_left_laser_scan_topic_launch_argument,
            declare_rear_right_laser_scan_topic_launch_argument,
            declare_virtual_laser_scan_topic_launch_argument,
            declare_virtual_laser_scan_frame_id_launch_argument,
            declare_virtual_laser_scan_angle_min_launch_argument,
            declare_virtual_laser_scan_angle_max_launch_argument,
            declare_virtual_laser_scan_angle_increment_launch_argument,
            declare_virtual_laser_scan_time_increment_launch_argument,
            declare_virtual_laser_scan_scan_time_launch_argument,
            declare_virtual_laser_scan_range_min_launch_argument,
            declare_virtual_laser_scan_range_max_launch_argument,
            declare_virtual_point_cloud_topic_launch_argument,
            declare_virtual_point_cloud_frame_id_launch_argument,
            launch_virtual_point_cloud_node,
            launch_virtual_laser_scan_node,
        ]
    )
