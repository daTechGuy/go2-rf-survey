"""Publish the robot description and TF tree.

    ros2 launch go2_survey_description description.launch.py

Phase 3 verification -- run this, then in RViz set the fixed frame to
base_footprint and add a TF display. Confirm:

* livox_frame sits forward and above base_link, visibly pitched nose-down
* camera_depth_optical_frame has +Z pointing forward out of the robot
* base_footprint is on the ground plane

Then add the LiDAR cloud. The floor must read flat and at z = 0, and walls
must be vertical. A sloping floor means mount_pitch_deg is wrong; a flat but
offset floor means lidar_z or stand_height is wrong.

Override any measured value without editing files:

    ros2 launch go2_survey_description description.launch.py \\
        mount_pitch_deg:=25.0 lidar_z:=0.12
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Keep in step with config/mount.yaml. Values there are the record; these are
# the fallbacks so the launch file works standalone.
GEOMETRY_ARGS = [
    ("mount_pitch_deg", "22.0", "Mid-360 nose-down pitch, degrees"),
    ("lidar_x", "0.16", "metres forward of trunk origin"),
    ("lidar_y", "0.0", "metres left of trunk origin"),
    ("lidar_z", "0.10", "metres above trunk origin"),
    ("cam_x", "0.29", "D435i, metres forward"),
    ("cam_y", "0.0", "D435i, metres left"),
    ("cam_z", "0.02", "D435i, metres up"),
    ("cam_pitch_deg", "0.0", "D435i nose-down pitch, degrees"),
    ("stand_height", "0.32", "trunk height while standing, metres"),
]


def generate_launch_description() -> LaunchDescription:
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in GEOMETRY_ARGS
    ]
    declared.append(
        DeclareLaunchArgument("rviz", default_value="false", description="open RViz")
    )
    declared.append(
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="use /clock"
        )
    )

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("go2_survey_description"), "urdf", "go2_survey.urdf.xacro"]
    )

    # Each geometry arg is forwarded to xacro as name:=value.
    xacro_cmd = ["xacro ", xacro_file]
    for name, _, _ in GEOMETRY_ARGS:
        xacro_cmd += [" ", name, ":=", LaunchConfiguration(name)]

    robot_description = {
        "robot_description": Command(xacro_cmd),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("go2_survey_description"),
                "rviz",
                "survey.rviz",
            ),
        ],
    )

    return LaunchDescription([*declared, rsp, rviz])
