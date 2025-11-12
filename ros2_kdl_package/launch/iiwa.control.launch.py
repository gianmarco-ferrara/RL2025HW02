import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "ros2_kdl_package"
    config_file = os.path.join(
        get_package_share_directory(pkg_name), "config", "parameters.yaml"
    )

    ctrl_arg = DeclareLaunchArgument(
        "ctrl",
        default_value="velocity_ctrl",
        description="Controller type (velocity_ctrl | velocity_ctrl_null)",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=config_file,
        description="Path to parameters YAML file",
    )

    cmd_interface_arg = DeclareLaunchArgument(
        "cmd_interface",
        default_value="position",
        description="Command interface (position | velocity | effort)",
    )

    ros2_kdl_node = Node(
        package=pkg_name,
        executable="ros2_kdl_node",
        name="ros2_kdl_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_file"),
            {"ctrl": LaunchConfiguration("ctrl")},
            {"cmd_interface": LaunchConfiguration("cmd_interface")},
        ],
    )

    return LaunchDescription(
        [params_file_arg, ctrl_arg, cmd_interface_arg, ros2_kdl_node]
    )
