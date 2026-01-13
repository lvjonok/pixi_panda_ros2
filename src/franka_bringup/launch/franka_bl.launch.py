#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

"""
Franka robot bringup launch file using better_launch.

This replicates the functionality of franka.launch.py using the better_launch framework.

Usage:
    bl franka_bringup franka_bl.launch.py robot_ip=<ROBOT_IP>
    # or
    python3 franka_bl.launch.py robot_ip=<ROBOT_IP>
"""

import subprocess
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this


def get_robot_description(
    description_package: str,
    description_file: str,
    load_gripper: bool,
    robot_ip: str,
    use_fake_hardware: bool,
    fake_sensor_commands: bool,
) -> str:
    """Process xacro file and return robot description URDF string."""
    package_path = get_package_share_directory(description_package)
    xacro_file = Path(package_path) / description_file

    # Build xacro command with arguments
    cmd = [
        "xacro",
        str(xacro_file),
        f"hand:={'true' if load_gripper else 'false'}",
        f"robot_ip:={robot_ip}",
        f"use_fake_hardware:={'true' if use_fake_hardware else 'false'}",
        f"fake_sensor_commands:={'true' if fake_sensor_commands else 'false'}",
    ]

    result = subprocess.run(cmd, capture_output=True, text=True, check=True)
    return result.stdout


@launch_this
def franka_bringup(
    robot_ip: str,
    description_package: str = "franka_description",
    description_file: str = "robots/real/panda_arm.urdf.xacro",
    load_gripper: bool = True,
    use_fake_hardware: bool = False,
    fake_sensor_commands: bool = False,
    use_rviz: bool = False,
):
    # Launch the Franka robot with ros2_control.
    # Args:
    #   robot_ip: Hostname or IP address of the robot.
    #   description_package: Description package with robot URDF/xacro files.
    #   description_file: Path to robot description file relative to description package.
    #   load_gripper: Use Franka Gripper as an end-effector.
    #   use_fake_hardware: Use fake hardware instead of real robot.
    #   fake_sensor_commands: Fake sensor commands (only valid when use_fake_hardware is true).
    #   use_rviz: Visualize the robot in Rviz.
    bl = BetterLaunch()

    # Get robot description from xacro
    robot_description = get_robot_description(
        description_package=description_package,
        description_file=description_file,
        load_gripper=load_gripper,
        robot_ip=robot_ip,
        use_fake_hardware=use_fake_hardware,
        fake_sensor_commands=fake_sensor_commands,
    )

    # Path to controllers config
    franka_bringup_share = get_package_share_directory("franka_bringup")
    controllers_file = str(Path(franka_bringup_share) / "config" / "controllers.yaml")

    # Robot state publisher
    bl.node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        params={"robot_description": robot_description},
    )

    # Joint state publisher (aggregates joint states from multiple sources)
    bl.node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        params={
            "source_list": ["franka/joint_states", "panda_gripper/joint_states"],
            "rate": 30,
        },
    )

    # Franka control node - main hardware interface
    # Load controller parameters
    controller_params = bl.load_params(controllers_file)
    bl.node(
        package="franka_control2",
        executable="franka_control2_node",
        params=[{"robot_description": robot_description}, controller_params],
        remaps={"joint_states": "franka/joint_states"},
        on_exit=lambda: sys.exit(0),  # Exit when control node terminates
    )

    # Spawn joint state broadcaster (active)
    bl.node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        cmd_args=["joint_state_broadcaster", "--param-file", controllers_file],
    )

    # Spawn franka robot state broadcaster (only for real hardware)
    if not use_fake_hardware:
        bl.node(
            package="controller_manager",
            executable="spawner",
            name="franka_robot_state_broadcaster_spawner",
            cmd_args=["franka_robot_state_broadcaster", "--param-file", controllers_file],
        )

    # Spawn pose broadcaster (active)
    bl.node(
        package="controller_manager",
        executable="spawner",
        name="pose_broadcaster_spawner",
        cmd_args=["pose_broadcaster", "--param-file", controllers_file],
    )

    # Spawn inactive controllers (crisp_controllers)
    inactive_controllers = [
        "joint_trajectory_controller",
        "cartesian_impedance_controller",
        "joint_impedance_controller",
        "gravity_compensation",
    ]

    for controller in inactive_controllers:
        bl.node(
            package="controller_manager",
            executable="spawner",
            name=f"{controller}_spawner",
            cmd_args=[controller, "--inactive", "--param-file", controllers_file],
        )

    # Include gripper launch (conditionally)
    if load_gripper:
        bl.include(
            package="franka_gripper",
            launchfile="gripper.launch.py",
            robot_ip=robot_ip,
            use_fake_hardware=str(use_fake_hardware).lower(),
        )

    # Start RViz (conditionally)
    if use_rviz:
        description_share = get_package_share_directory(description_package)
        rviz_config = str(Path(description_share) / "rviz" / "visualize_franka.rviz")
        bl.node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            cmd_args=["--display-config", rviz_config],
        )
