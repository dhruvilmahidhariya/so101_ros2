"""
Dual launch: Gazebo + real robot + MoveIt + RViz.
Same as so101_gazebo_moveit but trajectories are mirrored to both the sim and the real
robot via a trajectory_mirror node. Start the real robot controller_manager (with
so_arm_100_hardware) separately before or in another terminal; it should expose
arm_controller and gripper_controller under /real_controller_manager.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_lerobot_description = get_package_share_directory("lerobot_description")
    pkg_lerobot_moveit = get_package_share_directory("lerobot_moveit")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    model_path = os.path.join(pkg_lerobot_description, "urdf", "so101.urdf.xacro")
    ros2_control_config = os.path.join(
        pkg_lerobot_moveit, "config", "ros2_controllers.yaml"
    )
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                model_path,
                " robot_name:=so101_new_calib",
                " use_primitive_collision:=true",
                " ros2_control_config_file:=",
                ros2_control_config,
            ]
        ),
        value_type=str,
    )

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(pkg_lerobot_description).parent.resolve())],
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True}
        ],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[("gz_args", " -v 4 -r empty.sdf ")],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "so101"],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    spawn_kinematics = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["kinematics_controller"],
        output="screen",
    )
    spawn_gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[
            spawn_joint_state_broadcaster,
            spawn_kinematics,
            spawn_gripper,
        ],
    )

  

    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="lerobot_moveit")
        .to_moveit_configs()
    )
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": moveit_config.move_group_capabilities["capabilities"],
        "disable_capabilities": moveit_config.move_group_capabilities[
            "disable_capabilities"
        ],
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }
    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        {"robot_description": robot_description, "use_sim_time": True},
    ]
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
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
    )

    rviz_config = str(moveit_config.package_path / "config" / "moveit.rviz")
    rviz_params = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {"use_sim_time": True},
    ]
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=rviz_params,
    )

    return LaunchDescription([
        gz_resource_path,
        rsp_node,
        gazebo_launch,
        spawn_entity,
        clock_bridge,
        delayed_controllers,

        move_group_node,
        rviz_node,
    ])
