# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the package directory
    bringup_dir = get_package_share_directory("pb2025_sentry_behavior")
    package_prefix = get_package_prefix("pb2025_sentry_behavior")
    workspace_install_dir = os.path.dirname(package_prefix)

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_fake_referee = LaunchConfiguration("use_fake_referee")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    overlay_lib_dirs = [
        os.path.join(package_prefix, "lib"),
        os.path.join(workspace_install_dir, "robot_msgs", "lib"),
        os.path.join(workspace_install_dir, "auto_aim_interfaces", "lib"),
        os.path.join(workspace_install_dir, "btcpp_ros2_interfaces", "lib"),
    ]
    existing_overlay_lib_dirs = [
        lib_dir for lib_dir in overlay_lib_dirs if os.path.isdir(lib_dir)
    ]
    current_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    local_ld_library_path = os.pathsep.join(existing_overlay_lib_dirs)
    if current_ld_library_path:
        local_ld_library_path = (
            f"{local_ld_library_path}{os.pathsep}{current_ld_library_path}"
        )
    ld_library_path_envvar = SetEnvironmentVariable(
        "LD_LIBRARY_PATH", local_ld_library_path
    )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_use_fake_referee_cmd = DeclareLaunchArgument(
        "use_fake_referee",
        default_value="false",
        description="Start fake_referee node if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "sentry_behavior.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    fake_referee_node = Node(
        package="pb2025_sentry_behavior",
        executable="fake_referee.py",
        name="fake_referee",
        output="screen",
        condition=IfCondition(use_fake_referee),
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            Node(
                package="pb2025_sentry_behavior",
                executable="pb2025_sentry_behavior_server",
                name="bt_action_server",
                output="screen",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="pb2025_sentry_behavior",
                executable="pb2025_sentry_behavior_client",
                name="sentry_behavior_client",
                output="screen",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="pb2025_sentry_behavior",
                executable="mapping_bag_recorder.py",
                name="mapping_bag_recorder",
                output="screen",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            fake_referee_node,
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)
    ld.add_action(ld_library_path_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_fake_referee_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch the nodes
    ld.add_action(bringup_cmd_group)

    return ld
