"""
Single launch for SO101: sim, real, or both.
  robot_mode:=sim   -> Gazebo + MoveIt + RViz (fake hardware)
  robot_mode:=real  -> Real robot + MoveIt + RViz
  robot_mode:=both  -> Gazebo + real robot + mirror + MoveIt + RViz (both move together)
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_lerobot_description = get_package_share_directory("lerobot_description")
    pkg_lerobot_moveit = get_package_share_directory("lerobot_moveit")
    pkg_lerobot_controller = get_package_share_directory("lerobot_controller")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    robot_mode_arg = DeclareLaunchArgument(
        "robot_mode",
        default_value="sim",
        description="sim (Gazebo only) | real (real robot only) | both (Gazebo + real + mirror)",
    )
    robot_mode = LaunchConfiguration("robot_mode", default="sim")

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyACM0",
        description="Serial port for real hardware (used when robot_mode is real or both)",
    )
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyACM0")

    # ----- Robot descriptions -----
    model_path = os.path.join(pkg_lerobot_description, "urdf", "so101.urdf.xacro")
    ros2_control_config = os.path.join(pkg_lerobot_moveit, "config", "ros2_controllers.yaml")
    calibration_file = os.path.join(pkg_lerobot_controller, "config", "calibration_so101_real.yaml")

    robot_description_sim = ParameterValue(
        Command(
            [
                "xacro ", model_path,
                " robot_name:=so101_new_calib use_primitive_collision:=true",
                " ros2_control_config_file:=", ros2_control_config,
            ]
        ),
        value_type=str,
    )

    robot_description_real = ParameterValue(
        Command(
            [
                "xacro ", model_path,
                " use_sim:=false robot_name:=so101_new_calib",
                " calibration_file:=", calibration_file,
                " serial_port:=", serial_port,
            ]
        ),
        value_type=str,
    )

    # Conditions: sim only, real only, both
    real_only = EqualsSubstitution(robot_mode, "real")
    both_mode = EqualsSubstitution(robot_mode, "both")
    sim_only_cond = IfCondition(EqualsSubstitution(robot_mode, "sim"))
    real_only_cond = IfCondition(real_only)
    both_mode_cond = IfCondition(both_mode)
    sim_or_both_cond = UnlessCondition(real_only)  # sim or both

    # ----- Sim block (Gazebo + spawn + sim controllers): when sim or both -----
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(pkg_lerobot_description).parent.resolve())],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[("gz_args", " -v 4 -r empty.sdf ")],
        condition=sim_or_both_cond,
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "so101"],
        condition=sim_or_both_cond,
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        condition=sim_or_both_cond,
    )

    spawn_joint_state_sim = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
        condition=sim_or_both_cond,
    )
    spawn_kinematics_sim = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["kinematics_controller"],
        output="screen",
        condition=sim_or_both_cond,
    )
    spawn_gripper_sim = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
        condition=sim_or_both_cond,
    )
    delayed_sim_controllers = TimerAction(
        period=5.0,
        actions=[spawn_joint_state_sim, spawn_kinematics_sim, spawn_gripper_sim],
        condition=sim_or_both_cond,
    )

    # Real controller launch: when real only (name=controller_manager) or both (name=real_controller_manager)
    real_launch_controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lerobot_controller, "launch", "so101_real_controller.launch.py")
        ),
        launch_arguments=[
            ("controller_manager_name", "controller_manager"),
            ("serial_port", serial_port),
        ],
        condition=real_only_cond,
    )
    real_launch_real_controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lerobot_controller, "launch", "so101_real_controller.launch.py")
        ),
        launch_arguments=[
            ("controller_manager_name", "real_controller_manager"),
            ("publish_robot_state", "false"),
            ("serial_port", serial_port),
        ],
        condition=both_mode_cond,
    )

    # RSP for real-only mode (real launch includes its own RSP; for real_only we rely on that)
    # So we don't add another RSP for real. For sim/both we need RSP with sim description.
    rsp_sim_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_sim, "use_sim_time": True}],
        condition=sim_or_both_cond,
    )



    # MoveIt
    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="lerobot_moveit")
        .to_moveit_configs()
    )
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": moveit_config.move_group_capabilities["capabilities"],
        "disable_capabilities": moveit_config.move_group_capabilities["disable_capabilities"],
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    move_group_sim = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_configuration,
            {"robot_description": robot_description_sim, "use_sim_time": True},
        ],
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
        condition=sim_only_cond,
    )

    move_group_real_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_configuration,
            {"robot_description": robot_description_real, "use_sim_time": False},
        ],
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )
    # Delay move_group and rviz in real mode so controller_manager gets real robot_description first
    move_group_real = TimerAction(
        period=4.0,
        actions=[move_group_real_node],
        condition=real_only_cond,
    )

    move_group_both = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_configuration,
            {"robot_description": robot_description_sim, "use_sim_time": True},
        ],
        remappings=[
            (
                "controller_manager/kinematics_controller/follow_joint_trajectory",
                "trajectory_mirror/kinematics_controller/follow_joint_trajectory",
            ),
            (
                "controller_manager/gripper_controller/follow_joint_trajectory",
                "trajectory_mirror/gripper_controller/follow_joint_trajectory",
            ),
        ],
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
        condition=both_mode_cond,
    )

    rviz_config = str(moveit_config.package_path / "config" / "moveit.rviz")
    rviz_params_sim = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {"use_sim_time": True},
    ]
    rviz_params_real = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {"use_sim_time": False},
    ]

    rviz_both = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=rviz_params_sim,
        condition=both_mode_cond,
    )
    rviz_real_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=rviz_params_real,
    )
    rviz_real = TimerAction(
        period=4.0,
        actions=[rviz_real_node],
        condition=real_only_cond,
    )

    rviz_sim_only = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=rviz_params_sim,
        condition=IfCondition(EqualsSubstitution(robot_mode, "sim")),
    )

    return LaunchDescription([
        robot_mode_arg,
        serial_port_arg,
        gz_resource_path,
        rsp_sim_node,
        gazebo_launch,
        spawn_entity,
        clock_bridge,
        delayed_sim_controllers,
        real_launch_controller_manager,
        real_launch_real_controller_manager,
        move_group_sim,
        move_group_real,
        move_group_both,
        rviz_sim_only,
        rviz_real,
        rviz_both,
    ])
