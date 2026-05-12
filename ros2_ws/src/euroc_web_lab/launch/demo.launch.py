from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    save_dir = LaunchConfiguration("save_dir")
    port = LaunchConfiguration("port")
    bag_path = LaunchConfiguration("bag_path")
    bag_rate = LaunchConfiguration("bag_rate")
    manage_bag_play = LaunchConfiguration("manage_bag_play")

    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/cam0/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/cam0/camera_info"),
            DeclareLaunchArgument("save_dir", default_value="/data/saved"),
            DeclareLaunchArgument("port", default_value="8080"),
            DeclareLaunchArgument("bag_path", default_value="/external/euroc/MH_01_easy_ros2_ready"),
            DeclareLaunchArgument("bag_rate", default_value="0.5"),
            DeclareLaunchArgument("manage_bag_play", default_value="true"),
            Node(
                package="euroc_web_lab",
                executable="web_bridge_node",
                name="euroc_image_web_bridge",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "camera_info_topic": camera_info_topic,
                        "save_dir": save_dir,
                        "port": port,
                        "bag_path": bag_path,
                        "bag_rate": bag_rate,
                        "manage_bag_play": manage_bag_play,
                    }
                ],
            ),
        ]
    )
