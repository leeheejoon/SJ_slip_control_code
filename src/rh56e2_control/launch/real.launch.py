"""Activate one RH56E2 hand, initialize MOVEJ; hand commands are started manually."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    inspire_share = get_package_share_directory("inspire_description")
    xacro_file = os.path.join(inspire_share, "xacro", "ros2_control", "hand.xacro")
    controller_yaml = os.path.join(
        inspire_share, "config", "ros2_control", "RH56E2.yaml"
    )

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", xacro_file,
            " type:=RH56E2",
            " name:=",
            " direction:=", LaunchConfiguration("direction"),
            " ros2_control_hardware_type:=real",
            " gazebo:=false",
            " serial_port:=", LaunchConfiguration("serial_port"),
            " baudrate:=", LaunchConfiguration("baudrate"),
            " read_feedback:=true",
            " read_tactile:=", LaunchConfiguration("read_tactile"),
            " tactile_period_ms:=", LaunchConfiguration("tactile_period_ms"),
            " tactile_feedback_period_ms:=", LaunchConfiguration("tactile_feedback_period_ms"),
            " tactile_regions:=", LaunchConfiguration("tactile_regions"),
        ]),
        value_type=str,
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        output="screen",
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_yaml, {"use_sim_time": False}],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )
    state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )
    hand_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "hand_joint_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )
    fsm_init = Node(
        package="rh56e2_control",
        executable="fsm_init_node",
        output="screen",
    )
    def after_hand_controller(event, context):
        if event.returncode != 0:
            return [LogInfo(msg="Hand controller activation failed; FSM was not started.")]
        return [fsm_init]

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/ttyUSB0",
            description="USB-RS485 device path (a /dev/serial/by-id path is also accepted).",
        ),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("read_tactile", default_value="true", choices=["true", "false"],
                              description="Read RH56E2 T1 resistive tactile arrays."),
        DeclareLaunchArgument("tactile_regions",
                              default_value="index_pad,middle_pad,thumb_distal",
                              description="Comma-separated T1 regions, or all for all 17."),
        DeclareLaunchArgument("tactile_feedback_period_ms", default_value="100",
                              description="Feedback polling interval while tactile is enabled (20..200 ms)."),
        DeclareLaunchArgument("tactile_period_ms", default_value="0",
                              description="Extra wait after each tactile chunk (>=0 ms)."),
        DeclareLaunchArgument(
            "direction", default_value="-1", choices=["-1", "1"],
            description="-1: right hand; 1: left hand. Factory Modbus ID is 1 for either side.",
        ),
        RegisterEventHandler(OnProcessExit(
            target_action=hand_controller, on_exit=after_hand_controller,
        )),
        state_publisher,
        control_node,
        state_broadcaster,
        hand_controller,
    ])
