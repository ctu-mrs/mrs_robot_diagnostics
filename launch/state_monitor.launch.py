#!/usr/bin/env python3

import launch
import os
import sys

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    IfElseSubstitution,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_robot_diagnostics"

    this_pkg_path = get_package_share_directory(pkg_name)
    namespace = 'state_monitor'

    # #{ uav_name

    robot_name = LaunchConfiguration('robot_name')

    ld.add_action(DeclareLaunchArgument(
        'robot_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    uav_type = LaunchConfiguration('uav_type')
    ld.add_action(DeclareLaunchArgument(
        'uav_type',
        default_value=os.getenv('UAV_TYPE', "x500"),
        description="The uav type used for selecting platform configuration.",
    ))

    robot_type = LaunchConfiguration('robot_type')
    ld.add_action(DeclareLaunchArgument(
        'robot_type',
        default_value=os.getenv('ROBOT_TYPE', "multirotor"),
        description="The robot type used for selecting platform configuration.",
    ))

    available_sensors = LaunchConfiguration('available_sensors')
    ld.add_action(DeclareLaunchArgument(
        'available_sensors',
        default_value=os.getenv('AVAILABLE_SENSORS', "pixhawk,garmin"),
        description="The available sensors on the platform.",
    ))

    # #} end of custom_config

    # #{ standalone

    standalone = LaunchConfiguration('standalone')

    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='true',
        description='Whether to start a as a standalone or load into an existing container.'
    )

    ld.add_action(declare_standalone)

    # #} end of standalone

    # #{ container_name

    container_name = LaunchConfiguration('container_name')

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    )

    ld.add_action(declare_container_name)

    # #} end of container_name

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
        condition=PythonExpression(['"', custom_config, '" != "" and ',
                                   'not "', custom_config, '".startswith("/")']),
        if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
        else_value=custom_config
    )

    # #} end of custom_config

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', "false"),
        description="Should the node subscribe to sim time?",
    ))

    # #} end of custom_config

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    # #} end of log_level

    # #{ state monitor node

    state_monitor_node = ComposableNode(

        package=pkg_name,
        plugin='mrs_robot_diagnostics::state_monitor::StateMonitor',
        namespace=robot_name,
        name='state_monitor',

        parameters=[
            {"robot_name": robot_name},
            {"uav_type": uav_type},
            {"robot_type": robot_type},
            {"available_sensors": available_sensors},
            {"custom_config": custom_config},
            {"use_sim_time": use_sim_time},
            {'config': this_pkg_path + '/config/state_monitor_config.yaml'},
        ],

        remappings=[

            # publishers
            ("~/out/general_robot_info", "general_robot_info"),
            ("~/out/state_estimation_info", "state_estimation_info"),
            ("~/out/control_info", "control_info"),
            ("~/out/collision_avoidance_info", "collision_avoidance_info"),
            ("~/out/uav_info", "uav_info"),
            ("~/out/system_health_info", "system_health_info"),
            ("~/out/uav_state", "uav_state"),
            ("~/out/sensor_info", "sensor_info"),
            # subscribers
            ("~/in/automatic_start_can_takeoff", "automatic_start/can_takeoff"),
            ("~/in/battery_state", "hw_api/battery_state"),
            ("~/in/estimation_diagnostics", "estimation_manager/diagnostics"),
            ("~/in/hw_api_gnss", "hw_api/gnss"),
            ("~/in/hw_api_mag_heading", "hw_api/mag_heading"),
            ("~/in/control_manager_heading", "control_manager/heading"),
            ("~/in/control_manager_diagnostics", "control_manager/diagnostics"),
            ("~/in/control_manager_thrust", "control_manager/thrust"),
            ("~/in/mpc_tracker_diagnostics", "control_manager/mpc_tracker/diagnostics"),
            ("~/in/hw_api_status", "hw_api/status"),
            ("~/in/uav_status", "mrs_uav_status/uav_status"),
            ("~/in/mass_nominal", "control_manager/mass_nominal"),
            ("~/in/mass_estimate", "control_manager/mass_estimate"),
            ("~/in/hw_api_magnetic_field", "hw_api/magnetic_field"),

            ("~/in/errors", "errors"),
        ],
    )

    load_into_existing = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[state_monitor_node],
        condition=UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # #} end of state monitor node

    # #{ standalone container

    standalone_container = ComposableNodeContainer(
        namespace=robot_name,
        name=namespace + '_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output="screen",
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],
        composable_node_descriptions=[state_monitor_node],
        parameters=[
            {'use_intra_process_comms': True},
            {'thread_num': os.cpu_count()},
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(standalone)
    )

    ld.add_action(standalone_container)

    # #} end of own container

    return ld
