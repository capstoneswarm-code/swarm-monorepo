"""Launch the single physical robot in live-SLAM exploration mode."""

import math
import os
from pathlib import Path
import sys

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


FOLDER = Path(__file__).resolve().parent


def configured_params(defaults, radius):
    config = yaml.safe_load(defaults.read_text())
    for name in ("local_costmap", "global_costmap"):
        params = config[name][name]["ros__parameters"]
        clearance = radius + 0.12 if name == "global_costmap" else radius
        params["robot_radius"] = clearance
        if name == "global_costmap":
            params["obstacle_layer"]["footprint_clearing_enabled"] = False
        params["footprint_padding"] = 0.01
        params["inflation_layer"]["inflation_radius"] = clearance + 0.15
    config["collision_monitor"]["ros__parameters"]["StopZone"]["radius"] = radius + 0.07
    return config


def bringup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    radius = float(arg("robot_radius"))
    if not math.isfinite(radius) or not 0.03 <= radius <= 1.0:
        raise ValueError("Use a measured footprint radius in meters between 0.03 and 1.0")
    hardware = Path(arg("hardware_config")).expanduser().resolve()
    if not hardware.is_file():
        raise RuntimeError(f"Hardware configuration not found: {hardware}")

    mapping = FOLDER / "mapping.launch.py"
    if not mapping.is_file():
        raise RuntimeError(f"Missing your existing live-SLAM launch: {mapping}")
    config = configured_params(FOLDER / "nav2_params.yaml", radius)
    params = FOLDER / "nav2.generated.yaml"
    params.write_text(yaml.safe_dump(config, sort_keys=False))

    bridge = ExecuteProcess(
        cmd=[sys.executable, "-u", str(FOLDER / "robot_demo.py"), "--nav2",
             "--domain-id", os.environ.get("ROS_DOMAIN_ID", "78"),
             "--config", str(hardware)],
        output="screen", name="robot_hardware",
    )
    explorer = ExecuteProcess(
        cmd=[sys.executable, "-u", str(FOLDER / "frontier_explorer.py"),
             "--radius", str(radius + 0.12)],
        output="screen", name="frontier_explorer",
        condition=IfCondition(LaunchConfiguration("explore")),
    )

    # Wire the command chain explicitly. The hardware listens only to
    # Collision Monitor's final output, never to raw controller commands.
    raw_command = [("cmd_vel", "/nav2_cmd_vel")]
    specs = [
        ("nav2_planner", "planner_server", "planner_server", []),
        ("nav2_controller", "controller_server", "controller_server", raw_command),
        ("nav2_behaviors", "behavior_server", "behavior_server", raw_command),
        ("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother", raw_command),
        ("nav2_collision_monitor", "collision_monitor", "collision_monitor", []),
        ("nav2_bt_navigator", "bt_navigator", "bt_navigator", []),
    ]
    nodes = [Node(package=package, executable=executable, name=name,
                  parameters=[str(params)], remappings=remaps, output="screen")
             for package, executable, name, remaps in specs]

    return [
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason="Robot hardware bridge exited"))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=explorer,
            on_exit=[EmitEvent(event=Shutdown(reason="Frontier explorer exited"))],
        )),
        bridge,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(mapping))),
        *nodes,
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_nav", output="screen", parameters=[{
                 "use_sim_time": False, "autostart": True,
                 "node_names": [name for _, _, name, _ in specs],
             }]),
        explorer,
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("robot_radius", description="Measured footprint radius in meters"),
        DeclareLaunchArgument("explore", default_value="true",
                              description="Automatically send reachable frontier goals"),
        DeclareLaunchArgument("hardware_config", default_value=str(FOLDER / "hardware.yaml")),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=bringup)])
