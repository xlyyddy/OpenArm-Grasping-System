import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
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
    MOVEIT_CONFIG_PKG = "openarm_moveit_config"
    URDF_FILE_NAME = "openarm.urdf.xacro"
    SRDF_FILE_NAME = "openarm.srdf" 
    ROS2_CONTROLLERS_YAML = "ros2_controllers.yaml"
    USE_SIM_TIME = True

    # ==========================================
    # 2. 解析 URDF 和 SRDF（关键：确保初始位置正确传递）
    # ==========================================
    xacro_path = os.path.join(get_package_share_directory(MOVEIT_CONFIG_PKG), "config", URDF_FILE_NAME)
    
    # 关键修复：提供初始位姿文件的绝对路径
    initial_positions_file = os.path.join(
        get_package_share_directory(MOVEIT_CONFIG_PKG), "config", "initial_positions.yaml"
    )
    doc = xacro.process_file(xacro_path, mappings={"initial_positions_file": initial_positions_file})
    robot_description = {"robot_description": doc.toprettyxml(indent="  ")}

    srdf_path = os.path.join(get_package_share_directory(MOVEIT_CONFIG_PKG), "config", SRDF_FILE_NAME)
    srdf_doc = xacro.process_file(srdf_path) 
    robot_description_semantic = {"robot_description_semantic": srdf_doc.toprettyxml(indent="  ")}

    # ==========================================
    # 3. 加载配置文件
    # ==========================================
    kinematics_yaml = load_yaml(MOVEIT_CONFIG_PKG, "config/kinematics.yaml")
    
    joint_limits_yaml = load_yaml(MOVEIT_CONFIG_PKG, "config/joint_limits.yaml")
    if joint_limits_yaml and 'joint_limits' not in joint_limits_yaml:
        joint_limits_yaml = {'joint_limits': joint_limits_yaml}
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    # 关键修复：使用正确的 request_adapters 配置
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints default_planner_request_adapters/AddTimeOptimalParameterization""",
            "start_state_max_bounds_error": 0.1,
            "planner_configs": {
                "RRTConnectkConfigDefault": {"type": "geometric::RRTConnect", "range": 0.0},
            }
        }
    }

    moveit_controllers_yaml = load_yaml(MOVEIT_CONFIG_PKG, "config/moveit_controllers.yaml")
    trajectory_execution = moveit_controllers_yaml
    trajectory_execution['moveit_controller_manager'] = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    trajectory_execution['allowed_execution_duration_scaling'] = 1.2
    trajectory_execution['allowed_goal_duration_margin'] = 0.5
    trajectory_execution['allowed_start_tolerance'] = 0.05

    # 控制器配置文件路径
    ros2_controllers_path = os.path.join(
        get_package_share_directory(MOVEIT_CONFIG_PKG),
        "config",
        ROS2_CONTROLLERS_YAML,
    )
    
    # 关键修复：强制开启 open_loop_control，解决 arm 和 gripper 状态切换不一致问题
    # 参数覆盖字典 (Runtime Override)，用于覆盖 YAML 文件中的配置
    ros2_controllers_override = {
        "left_joint_trajectory_controller": {
            "ros__parameters": {
                "open_loop_control": True,  # 忽略状态反馈误差
                "constraints": {
                    "stopped_velocity_tolerance": 0.01,
                    "goal_time": 0.5  # 放宽目标到达时间限制
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

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # ==========================================
    # 4. 定义节点
    # ==========================================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": USE_SIM_TIME}],
    )

    # 关键修复：ros2_control_node 需要足够时间启动，输出设置为 screen 以便调试
    # 使用文件路径字符串（ROS2 会自动加载）并添加覆盖参数
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",  # 改为 screen 以便看到错误信息
        parameters=[
            robot_description,         # 需要 URDF 来解析 ros2_control 标签
            ros2_controllers_path,     # 加载原始配置文件 (定义了 type, joints 等)
            ros2_controllers_override, # 加载覆盖参数 (强制覆盖 open_loop_control)
            {"use_sim_time": USE_SIM_TIME},
        ],
        # 确保节点在失败时不会静默退出，并启用 respawn 以便自动重启
        respawn=True,
        respawn_delay=2.0,
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {"use_sim_time": USE_SIM_TIME}, 
        ],
    )

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
            {"use_sim_time": USE_SIM_TIME},
        ],
    )

    # ==========================================
    # 5. Spawners（关键：确保正确的启动顺序）
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

    # 使用事件驱动方式：等待 ros2_control_node 启动后再启动 spawner
    # 这样可以确保 controller_manager 服务已经可用
    # 增加延时以确保服务完全就绪
    # 关键修复：增加更长的延时，确保controller_manager服务完全启动
    delayed_jsb_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                # 等待 controller_manager 服务就绪，增加延时到 8 秒
                TimerAction(period=8.0, actions=[jsb_spawner]),
            ],
        )
    )

    delayed_arm_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(period=10.0, actions=[arm_spawner]),
            ],
        )
    )

    delayed_gripper_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(period=12.0, actions=[gripper_spawner]),
            ],
        )
    )

    # Move Group 和 RViz 等待控制器就绪
    # 关键修复：增加延时，确保所有控制器都已加载
    delayed_move_group = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(period=14.0, actions=[run_move_group_node]),
            ],
        )
    )

    delayed_rviz = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(period=15.0, actions=[rviz_node]),
            ],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        
        # 使用事件驱动方式确保正确的启动顺序
        delayed_jsb_spawner,
        delayed_arm_spawner,
        delayed_gripper_spawner,
        delayed_move_group,
        delayed_rviz,
    ])