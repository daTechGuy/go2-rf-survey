"""Launch the WiFi survey node.

Phase 5 (teleop deliverable):

    ros2 launch go2_survey_wifi survey.launch.py interface:=wlan1 \
        output_csv:=$HOME/surveys/floor1.csv

Phase 8 (stop-and-scan at waypoints):

    ros2 launch go2_survey_wifi survey.launch.py mode:=triggered
    # then, once the robot is at rest:
    ros2 service call /wifi_survey/scan_now std_srvs/srv/Trigger
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = [
    ("interface", "wlan1", "scanning adapter -- NOT the one carrying your SSH session"),
    ("use_sudo", "false", "invoke iw via sudo; prefer setcap, see scanner.py"),
    ("mode", "continuous", "continuous | triggered"),
    ("output_csv", "survey.csv", "appended to; header written once"),
    ("map_frame", "map", "fixed frame published by SLAM"),
    ("base_frame", "base_link", "robot body frame"),
    ("min_move_m", "1.0", "continuous mode: metres of travel between samples"),
    ("poll_period_s", "2.0", "continuous mode: how often to check for travel"),
    ("scan_timeout_s", "15.0", "raise if dual-band DFS sweeps time out"),
    ("max_tf_age_s", "1.0", "discard samples whose pose is staler than this"),
]


def generate_launch_description() -> LaunchDescription:
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in ARGS
    ]

    survey = Node(
        package="go2_survey_wifi",
        executable="survey_node",
        name="wifi_survey",
        output="screen",
        emulate_tty=True,
        parameters=[{name: LaunchConfiguration(name) for name, _, _ in ARGS}],
    )

    return LaunchDescription([*declared, survey])
