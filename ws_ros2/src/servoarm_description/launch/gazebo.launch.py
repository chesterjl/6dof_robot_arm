import os
import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

# ──────────────────────────────────────────────────────────────────────────────
#  PATHS & CONFIG
# ──────────────────────────────────────────────────────────────────────────────
rvizRelativePath        = "config/display.rviz"
ros2ControlRelativePath = "config/ros2_controller.yaml"
CONTROLLER_MANAGER_DELAY = 6.0   # seconds


def launch_setup(context, *args, **kwargs):
    """
    This function executes dynamically at launch execution runtime.
    """
    gui_str = LaunchConfiguration("gui").perform(context)
    use_empty_str = LaunchConfiguration("use_empty").perform(context)
    
    pkgPath = get_package_share_directory('servoarm_description')
    rvizConfigPath  = os.path.join(pkgPath, rvizRelativePath)
    ros2ControlPath = os.path.join(pkgPath, ros2ControlRelativePath)

    home_dir       = os.path.expanduser('~')
    xacroModelPath = os.path.join(
        home_dir,
        "ws_6dof_robot_arm/ws_ros2/src/servoarm_description/urdf/arm_main.xacro"
    )
    robot_desc        = xacro.process_file(xacroModelPath).toxml()
    robot_description = {'robot_description': robot_desc}

    # ── FIXED: Tell Gazebo where your custom package models are located ───────
    # This maps to your 'servoarm_description/model' folder layout
    package_models_path = os.path.join(pkgPath, "model")
    
    # Update environment variables for Gazebo Sim (Modern GZ utilizes GZ_SIM_RESOURCE_PATH)
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        os.environ["GZ_SIM_RESOURCE_PATH"] += f":{package_models_path}"
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] = package_models_path

    # Backwards compatibility fallback for older Ignition/Gazebo tracking variants
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] += f":{package_models_path}"
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = package_models_path

    # ── Resolve Which World File to Load ──────────────────────────────────────
    if use_empty_str.lower() == "true":
        gz_world_arg = "empty.sdf"
    else:
        gz_world_arg = os.path.join(pkgPath, "worlds", "pick_and_place_demo.world")

    # ── Gazebo Configurations ─────────────────────────────────────────────────
    if gui_str.lower() == "true":
        gz_args_value = f" -r -v 3 {gz_world_arg}"
    else:
        gz_args_value = f" --headless-rendering -s -r -v 3 {gz_world_arg}"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", gz_args_value)],
    )

    # ── ROS ↔ Gazebo Bridge ───────────────────────────────────────────────────
    bridge_params = os.path.join(pkgPath, 'parameters', 'bridge_parameters.yaml')
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output="screen",
    )

    # ── Robot State Publisher ─────────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ── Spawn Robot Into Gazebo ───────────────────────────────────────────────
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name",  "6dof_mg996r_arm",
            "-allow_renaming", "true",
        ],
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rvizConfigPath],
    )

    # ── Controller Spawners ───────────────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    arm_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--param-file", ros2ControlPath,
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    gripper_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_action_controller",
            "--param-file", ros2ControlPath,
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # ── Custom Arm Publisher (arm_pkg) ────────────────────────────────────────
    arm_publisher_node = Node(
        package="arm_pkg",
        executable="arm_publisher_gazebo",
        output="screen",
        name="arm_controller_node",
    )

    # Step 1 – Wait after spawn, then load joint_state_broadcaster
    delayed_jsb_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                TimerAction(
                    period=CONTROLLER_MANAGER_DELAY,
                    actions=[joint_state_broadcaster_spawner],
                )
            ],
        )
    )

    # Step 2 – After joint_state_broadcaster finishes, load arm_controller
    load_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_control_spawner],
        )
    )

    # Step 3 – After arm_controller finishes, load gripper_action_controller
    load_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_control_spawner,
            on_exit=[gripper_control_spawner],
        )
    )

    # Step 4 – Wait until the gripper controller is ready before starting the node loop
    load_arm_publisher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_control_spawner,
            on_exit=[arm_publisher_node],
        )
    )

    return [
        gazebo,
        gazebo_bridge,
        robot_state_publisher_node,
        gz_spawn_entity,
        rviz_node,
        delayed_jsb_spawner,
        load_arm_controller,
        load_gripper_controller,
        load_arm_publisher,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            name="gui",
            default_value="true",
            description="Start Gazebo GUI (false = headless)."
        ),
        DeclareLaunchArgument(
            name="use_empty",
            default_value="true",
            description="If true, use empty.sdf. If false, load pick_and_place_demo.world"
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])