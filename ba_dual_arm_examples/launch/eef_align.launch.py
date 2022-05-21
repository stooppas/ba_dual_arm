from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command,PathJoinSubstitution,FindExecutable
from launch_ros.actions import Node

from niryo_one_moveit_config.launch_common import load_yaml,load_yaml_abs
import xacro
import os



def generate_launch_description():

    description_dir = get_package_share_directory("niryo_one_description")
    urdf_file = "niryo_two.urdf.xacro"
    moveit_config_package = "niryo_one_moveit_config"
    moveit_config_file = "niryo_two.srdf.xacro"

    joint_limit_params = PathJoinSubstitution(
        [get_package_share_directory(moveit_config_package), "config", "joint_limits.yaml"]
    )
    urdf_path = os.path.join(description_dir,"urdf",urdf_file)

    # "/home/pasto/ros2_ws/ba_ws/src/ba_dual_arm/niryo_one_moveit_config/urdf/ab_bot.xacro"
    
    robot_description = {"robot_description": Command(['xacro',' ', urdf_path])}

    srdf_path = os.path.join(get_package_share_directory(moveit_config_package),"srdf",moveit_config_file)

    robot_description_semantic = {"robot_description_semantic": Command(['xacro',' ', srdf_path])}

    robot_description_kinematics = PathJoinSubstitution(
        [get_package_share_directory(moveit_config_package), "config", "kinematics.yaml"]
    )

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("niryo_one_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("niryo_one_moveit_config", "config/controllers.yaml")

    move_group = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "allow_trajectory_execution":True,
        "trajectory_execution.execution_duration_monitoring":True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.0,
        "max_safe_path_cost":1,
        "jiggle_fraction":0.05,
    }

    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_planning_scene_hz":60.0,
        "plan_execution.record_trajectory_state_frequency":4.0
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="ba_dual_arm_examples",
        executable="eef_align",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            move_group,
            planning_scene_monitor,
        ],
        #arguments=['--ros-args', '--log-level', 'DEBUG']
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(move_group_node)    

    return ld   