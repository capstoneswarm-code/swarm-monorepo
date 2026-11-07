from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# REPLACE with the measured laser origin relative to base_link.
# Translation: meters. Yaw: radians.
# Assumes the LiDAR is horizontal and upright.
LIDAR_X = 0.0
LIDAR_Y = 0.0
LIDAR_Z = 0.0
LIDAR_YAW = 0.0

LIDAR_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)


def generate_launch_description():
    folder = Path(__file__).resolve().parent
    share = Path(get_package_share_directory("slam_toolbox"))

    config = yaml.safe_load(
        (share / "config/mapper_params_online_async.yaml").read_text()
    )

    config["slam_toolbox"]["ros__parameters"].update({
        "use_sim_time": False,
        "mode": "mapping",
        "odom_frame": "odom",
        "base_frame": "base_link",
        "map_frame": "map",
        "scan_topic": "/scan",
        "resolution": 0.05,
        "map_update_interval": 1.0,
        "minimum_time_interval": 0.1,
        "minimum_travel_distance": 0.05,
        "minimum_travel_heading": 0.05,
        "use_map_saver": True,
    })

    params = folder / "slam_mapping.generated.yaml"
    params.write_text(yaml.safe_dump(config, sort_keys=False))

    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser",
            output="screen",
            arguments=[
                "--x", str(LIDAR_X),
                "--y", str(LIDAR_Y),
                "--z", str(LIDAR_Z),
                "--yaw", str(LIDAR_YAW),
                "--pitch", "0",
                "--roll", "0",
                "--frame-id", "base_link",
                "--child-frame-id", "laser",
            ],
        ),

        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_node",
            output="screen",
            parameters=[{
                "channel_type": "serial",
                "serial_port": LIDAR_PORT,
                "serial_baudrate": 115200,
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
                "use_sim_time": False,
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / "launch/online_async_launch.py")
            ),
            launch_arguments={
                "slam_params_file": str(params),
                "use_sim_time": "false",
                "autostart": "true",
                "use_lifecycle_manager": "false",
            }.items(),
        ),
    ])
