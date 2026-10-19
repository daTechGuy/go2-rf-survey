"""Launch the /cmd_vel to sport-mode bridge.

    source ~/unitree_ros2/setup.sh
    ros2 launch go2_survey_bridge bridge.launch.py

Check it before trusting it to a planner. With the robot on a stand:

    ros2 service call /go2_cmd_bridge/balance_stand std_srvs/srv/Trigger
    ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \\
        '{linear: {x: 0.2}}'
    # then Ctrl-C the publisher: the watchdog must stop the robot within
    # cmd_timeout_s. If it does not, do not proceed to Nav2.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = [
    ("cmd_vel_topic", "/cmd_vel", "velocity command input"),
    ("max_vx", "0.6", "m/s forward/back ceiling"),
    ("max_vy", "0.4", "m/s lateral ceiling"),
    ("max_vyaw", "0.8", "rad/s yaw ceiling"),
    ("accel_xy", "0.8", "m/s^2 slew limit -- a quadruped stumbles on steps"),
    ("accel_yaw", "1.6", "rad/s^2 slew limit"),
    ("cmd_timeout_s", "0.3", "stop if no /cmd_vel arrives within this"),
    ("publish_rate_hz", "50.0", "sport mode wants a continuous stream"),
    ("auto_balance_stand", "true", "Move is ignored without BalanceStand first"),
    ("deadband_xy", "0.02", "below this, hold still -- this is the scan state"),
    ("deadband_yaw", "0.03", "as above, for yaw"),
]


def generate_launch_description() -> LaunchDescription:
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in ARGS
    ]

    bridge = Node(
        package="go2_survey_bridge",
        executable="bridge_node",
        name="go2_cmd_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[{name: LaunchConfiguration(name) for name, _, _ in ARGS}],
    )

    return LaunchDescription([*declared, bridge])
