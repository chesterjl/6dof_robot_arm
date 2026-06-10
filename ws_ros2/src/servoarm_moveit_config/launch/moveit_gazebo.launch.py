import os
import xacro
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import TimerAction

def generate_launch_description():
    # ── 1. DYNAMIC PATH RESOLUTION ───────────────────────────────────────────
    # Dynamically locate the installation directory of your description package
    pkgPath = get_package_share_directory('servoarm_description')
    
    # Update this to match your actual main top-level xacro filename
    main_xacro_name = "arm_main.xacro" 
    xacro_path = os.path.join(pkgPath, "urdf", main_xacro_name)
    
    ros2ControlPath = os.path.join(pkgPath, "config/ros2_controller.yaml")
    bridge_params = os.path.join(pkgPath, 'parameters', 'bridge_parameters.yaml')

    # Process xacro into clean robot description XML
    robot_desc = xacro.process_file(xacro_path).toxml()
    robot_description = {'robot_description': robot_desc}

    # ── 2. MOVEIT 2 CONFIGURATION CONFIG ─────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder("servo_6dof_arm", package_name="servoarm_moveit_config")
        .robot_description(file_path=os.path.join("urdf", main_xacro_name))
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    declared_arguments = [
        DeclareLaunchArgument(name="gui", default_value="true", description="Start the Gazebo Sim GUI.")
    ]
    gui = LaunchConfiguration("gui")

    # ── 3. GAZEBO SIMULATION NODES ───────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
        condition=launch.conditions.IfCondition(gui)
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments=[("gz_args", "--headless-rendering -s -r -v 3 empty.sdf")],
        condition=launch.conditions.UnlessCondition(gui)
    )

    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    # ── GRIPPER ATTACH/DETACH BRIDGE ─────────────────────────────────────────
    gz_gripper_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/gripper/attach@std_msgs/msg/Empty@gz.msgs.Empty',
            '/gripper/detach@std_msgs/msg/Empty@gz.msgs.Empty',
        ],
        output='screen'
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "/robot_description",
                   "-name", "6dof_servo_arm",  
                   "-allow_renaming", "true"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}]
    )

    # ── 4. ROS2 CONTROL CONTROLLER SPAWNERS ──────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"], # Removed leading slash
        output="screen"
    )

    arm_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--param-file", ros2ControlPath, "--controller-manager", "/controller_manager"], # Removed leading slash
        output="screen"
    )

    gripper_action_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller", "--param-file", ros2ControlPath, "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # ── 5. MOVEIT 2 CORE PLANNERS ────────────────────────────────────────────
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"start_state_max_bounds_error": 0.01}  
        ],
    )

    delayed_move_group = TimerAction(
        period=10.0,  # seconds — gripper spawns ~8-9s after launch in your chain
        actions=[run_move_group_node]
    )

    rviz_base = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(moveit_config.package_path, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,           
            moveit_config.robot_description_semantic,  
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,          
            moveit_config.joint_limits,                
            {"use_sim_time": True},
            {"start_state_max_bounds_error": 0.01}
        ],
    )

    # ── 6. SEQUENTIAL EVENT HANDLING ─────────────────────────────────────────
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    load_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_control_spawner],
        )
    )

    load_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
        target_action=arm_control_spawner,
        on_exit=[gripper_action_control_spawner],
        )
    )
            
    nodeList = [
        gazebo,
        gazebo_headless,
        gazebo_bridge,
        gz_gripper_bridge,       
        gz_spawn_entity,
        robot_state_publisher_node,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller,
        delayed_move_group,
        rviz_base,
    ]

    return LaunchDescription(declared_arguments + nodeList)