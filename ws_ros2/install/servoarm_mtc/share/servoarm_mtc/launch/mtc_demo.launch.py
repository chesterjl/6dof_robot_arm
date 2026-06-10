import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ── FIXED: Removed the dead 'moveit_combo_setter' line entirely
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_configs = (
        MoveItConfigsBuilder(
            robot_name="6dof_mg996r_arm",
            package_name="servoarm_moveit_config"
        )
        .robot_description()
        .robot_description_semantic()
        .trajectory_execution()
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    mtc_node = Node(
        package="servoarm_mtc",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_configs.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([mtc_node])