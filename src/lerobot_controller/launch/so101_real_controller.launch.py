"""
Launch the real SO101 robot via so_arm_100_hardware.
controller_manager_name: use "controller_manager" for real-only (MoveIt connects directly),
  or "real_controller_manager" for dual mode (with trajectory mirror).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    pkg_lerobot_description = get_package_share_directory("lerobot_description")
    pkg_lerobot_controller = get_package_share_directory("lerobot_controller")

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyACM0",
        description="Serial port for real hardware (e.g. /dev/ttyACM0 or /dev/ttyUSB0)",
    )
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyACM0")

    controller_manager_name_arg = DeclareLaunchArgument(
        "controller_manager_name",
        default_value="real_controller_manager",
        description="Node name for controller_manager: controller_manager (real only) or real_controller_manager (dual)",
    )
    controller_manager_name = LaunchConfiguration("controller_manager_name", default="real_controller_manager")

    publish_robot_state_arg = DeclareLaunchArgument(
        "publish_robot_state",
        default_value="true",
        description="Set false when used in 'both' mode (RSP already from sim).",
    )
    publish_robot_state = LaunchConfiguration("publish_robot_state", default="true")

    calibration_file = os.path.join(
        pkg_lerobot_controller, "config", "calibration_so101_real.yaml"
    )
    model_path = os.path.join(pkg_lerobot_description, "urdf", "so101.urdf.xacro")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                model_path,
                " use_sim:=false",
                " robot_name:=so101_new_calib",
                " calibration_file:=",
                calibration_file,
                " serial_port:=",
                serial_port,
            ]
        ),
        value_type=str,
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        condition=IfCondition(EqualsSubstitution(publish_robot_state, "true")),
    )

    # MoveIt/RViz expect a "world" frame; RSP only publishes base_link and below.
    static_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name=controller_manager_name,
        parameters=[
            {"robot_description": robot_description, "use_sim_time": False},
            os.path.join(pkg_lerobot_controller, "config", "so101_real_controllers.yaml"),
        ],
    )

    spawn_joint_state = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
        ],
    )
    spawn_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "kinematics_controller",
            "--controller-manager",
            controller_manager_name,
        ],
    )
    spawn_gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            controller_manager_name,
        ],
    )
    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn_joint_state, spawn_arm, spawn_gripper],
    )

    return LaunchDescription([
        serial_port_arg,
        controller_manager_name_arg,
        publish_robot_state_arg,
        rsp_node,
        static_world,
        controller_manager_node,
        delayed_spawn,
    ])
