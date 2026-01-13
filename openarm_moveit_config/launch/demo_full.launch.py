import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

# ==========================================
# 辅助函数
# ==========================================
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        print(f"[ERROR] Cannot find file: {absolute_file_path}")
        return None

def generate_launch_description():
    # ==========================================
    # 1. 基础配置
    # ==========================================
    DESCRIPTION_PKG = "openarm_description"
    BRINGUP_PKG = "openarm_bringup"
    MOVEIT_CONFIG_PKG = "openarm_moveit_config"

    URDF_FILE_NAME = "v10.urdf.xacro"
    SRDF_FILE_NAME = "openarm.srdf" 
    CONTROLLERS_YAML_NAME = "openarm_v10_bimanual_controllers.yaml"
    
    # 硬件参数 (保持 Fake Hardware = True)
    XACRO_ARGS = {
        "arm_type": "v10",
        "bimanual": "true",
        "use_fake_hardware": "true",
        "use_sim_hardware": "false",
        "ros2_control": "true",
        "left_can_interface": "can1",
        "right_can_interface": "can0",
        "arm_prefix": "" 
    }

    # ==========================================
    # 2. 解析 URDF 和 SRDF
    # ==========================================
    xacro_path = os.path.join(get_package_share_directory(DESCRIPTION_PKG), "urdf", "robot", URDF_FILE_NAME)
    doc = xacro.process_file(xacro_path, mappings=XACRO_ARGS)
    robot_description = {"robot_description": doc.toprettyxml(indent="  ")}

    srdf_path = os.path.join(get_package_share_directory(MOVEIT_CONFIG_PKG), "config", SRDF_FILE_NAME)
    srdf_doc = xacro.process_file(srdf_path) 
    robot_description_semantic = {"robot_description_semantic": srdf_doc.toprettyxml(indent="  ")}

    # ==========================================
    # 3. 加载配置文件
    # ==========================================
    
    # [A] Kinematics
    kinematics_yaml = load_yaml(MOVEIT_CONFIG_PKG, "config/kinematics.yaml")
    
    # [B] Joint Limits
    joint_limits_yaml = load_yaml(MOVEIT_CONFIG_PKG, "config/joint_limits.yaml")
    if joint_limits_yaml and 'joint_limits' not in joint_limits_yaml:
        joint_limits_yaml = {'joint_limits': joint_limits_yaml}
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    # [C] OMPL Planning (根据报错修正插件名称!)
    # 你的日志显示插件名前缀是 default_planner_request_adapters (planner vs planning)
    # 并且时间参数化用的是 AddTimeOptimalParameterization
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/ValidateWorkspaceBounds default_planner_request_adapters/CheckStartStateBounds default_planner_request_adapters/PadRequestAdapter default_planner_request_adapters/AddTimeOptimalParameterization""",
            "start_state_max_bounds_error": 0.1,
            "planner_configs": {
                "RRTConnectkConfigDefault": {"type": "geometric::RRTConnect", "range": 0.0},
            }
        }
    }

    # [D] MoveIt Controllers (MoveIt 端的配置)
    moveit_controllers_yaml = load_yaml(MOVEIT_CONFIG_PKG, "config/moveit_controllers.yaml")
    trajectory_execution = moveit_controllers_yaml
    trajectory_execution['moveit_controller_manager'] = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    trajectory_execution['allowed_execution_duration_scaling'] = 1.2
    trajectory_execution['allowed_goal_duration_margin'] = 0.5
    trajectory_execution['allowed_start_tolerance'] = 0.05

    # [E] ROS2 Controllers Config (文件路径)
    # 修复策略：直接传递文件路径，保证 namespace 结构正确，解决 "type param not defined" 错误
    ros2_controllers_path = os.path.join(
        get_package_share_directory(BRINGUP_PKG),
        "config", "v10_controllers", CONTROLLERS_YAML_NAME
    )
    
    # [E-Extra] 补充参数字典：强制开启 open_loop_control
    # 这会覆盖文件中的对应参数，解决 "Goal rejected" 问题
    ros2_controllers_override = {
        "left_joint_trajectory_controller": {
            "ros__parameters": {
                "open_loop_control": True,
                "constraints": {
                    "stopped_velocity_tolerance": 0.01,
                    "goal_time": 0.5
                }
            }
        },
        "right_joint_trajectory_controller": {
            "ros__parameters": {
                "open_loop_control": True,
                "constraints": {
                    "stopped_velocity_tolerance": 0.01,
                    "goal_time": 0.5
                }
            }
        }
    }

    # [F] Monitor
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # ==========================================
    # 4. 定义节点
    # ==========================================

    # [Node 1] Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # [Node 2] ROS2 Control Node
    # 关键修改：同时传递 robot_description, 原始yaml文件, 和 覆盖参数字典
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description, 
            ros2_controllers_path,     # 加载原始文件 (定义 type)
            ros2_controllers_override  # 加载补充参数 (定义 open_loop_control)
        ],
    )

    # [Node 3] Move Group
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            robot_description_planning,    # Joint Limits
            ompl_planning_pipeline_config, # OMPL + Correct Adapters
            trajectory_execution,          # Controllers + MoveIt Tolerances
            planning_scene_monitor_parameters,
            {"use_sim_time": False}, 
        ],
    )

    # [Node 4] RViz
    rviz_config_file = os.path.join(get_package_share_directory(MOVEIT_CONFIG_PKG), "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config, 
            {"robot_description_kinematics": kinematics_yaml},
        ],
    )

    # ==========================================
    # 5. Spawners
    # ==========================================
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_joint_trajectory_controller", "right_joint_trajectory_controller", "-c", "/controller_manager"],
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gripper_controller", "right_gripper_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        
        # 确保 JSB 先启动
        TimerAction(period=1.5, actions=[jsb_spawner]),
        
        # 确保 Arm Controller 在 JSB 之后启动
        TimerAction(period=3.5, actions=[arm_spawner]),
        
        # Gripper 最后启动
        TimerAction(period=4.5, actions=[gripper_spawner]),
        
        run_move_group_node,
        rviz_node
    ])