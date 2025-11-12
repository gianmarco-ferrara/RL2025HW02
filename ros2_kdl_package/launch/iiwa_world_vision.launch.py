import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.actions import TimerAction


def generate_launch_description():

    # Path to packages
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_iiwa_description = get_package_share_directory("iiwa_description")
    # Include Gazebo launch file
    aruco_model_sdf = os.path.join(
        pkg_iiwa_description, "gazebo", "worlds", "aruco_world.world"
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-r " + aruco_model_sdf,
            "publish_clock": "true",
        }.items(),
    )

    # Load robot description from xacro
    xacro_file_name = "iiwa.config.xacro"
    xacro = os.path.join(
        get_package_share_directory("iiwa_description"), "config", xacro_file_name
    )

    robot_description_param = {"robot_description": Command(["xacro ", xacro])}

    # Publish robot joint states to TF transforms
    load_robot_description_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param, {"use_sim_time": True}],
    )

    # Spawn robot entity in Gazebo
    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "/robot_description", "-name", "iiwa"],
    )

    # Clock bridge to keep simulation time and ROS2 time synchronized
    clock_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )
    # Camera bridge
    camera_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "--ros-args",
            "-r",
            "/camera:=/stereo/left/image_rect_color",
            "-r",
            "/camera_info:=/stereo/left/camera_info",
        ],
        output="screen",
    )

    service_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="set_pose_bridge",
        arguments=["/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose"],
        output="screen",
    )
    # Open RQT Image View to visualize ArUco detection results
    rqt_image_view_node = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
        output="screen",
        arguments=["/aruco_single/result"],
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "velocity_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    spawn_jsb_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[joint_state_broadcaster_node],
        )
    )

    spawn_controller_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_node,
            on_exit=[robot_controller_spawner],
        )
    )
    pkg_aruco_ros = get_package_share_directory("aruco_ros")

    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_aruco_ros, "launch", "single.launch.py")
        ),
        launch_arguments={"marker_size": "0.1", "marker_id": "14"}.items(),
    )

    return LaunchDescription(
        [
            gazebo_launch,
            load_robot_description_node,
            spawn_entity_node,
            aruco_launch,
            rqt_image_view_node,
            clock_bridge,
            camera_bridge,
            service_bridge_node,
            spawn_jsb_handler,
            spawn_controller_handler,
        ]
    )
