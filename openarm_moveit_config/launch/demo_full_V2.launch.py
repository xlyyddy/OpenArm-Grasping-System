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
    """
    通用函数：用于加载 YAML 配置文件
    :param package_name: ROS 2 功能包名称
    :param file_path: 包内的相对路径 (例如 'config/file.yaml')
    :return: 解析后的 Python 字典
    """
    # 获取功能包的绝对安装路径
    package_path = get_package_share_directory(package_name)
    # 拼接完整的文件路径
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        # 打开并安全加载 YAML 内容
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # 如果找不到文件，打印错误并返回 None
        print(f"[ERROR] Cannot find file: {absolute_file_path}")
        return None

def generate_launch_description():
    # ==========================================
    # 1. 基础配置 (定义包名和文件名)
    # ==========================================
    DESCRIPTION_PKG = "openarm_description"      # 存放 URDF/Xacro 的包
    BRINGUP_PKG = "openarm_bringup"              # 存放启动配置和控制器参数的包
    MOVEIT_CONFIG_PKG = "openarm_moveit_config"  # MoveIt 配置包 (SRDF, 运动学配置等)

    URDF_FILE_NAME = "v10.urdf.xacro"            # 机器人模型主文件
    SRDF_FILE_NAME = "openarm.srdf"              # 语义描述文件 (定义规划组、碰撞对)
    CONTROLLERS_YAML_NAME = "openarm_v10_bimanual_controllers.yaml" # 控制器参数文件
    
    # 硬件参数字典 (传递给 xacro 解析器)
    # 这里设置 use_fake_hardware=true 用于仿真/测试
    XACRO_ARGS = {
        "arm_type": "v10",                  # 机械臂型号
        "bimanual": "true",                 # 双臂配置
        "use_fake_hardware": "true",        # 启用虚拟硬件接口 (Mock Hardware)
        "use_sim_hardware": "false",        # 不使用 Gazebo 仿真硬件
        "ros2_control": "true",             # 启用 ros2_control 标签生成
        "left_can_interface": "can1",       # 左臂 CAN 接口名 (真实硬件用)
        "right_can_interface": "can0",      # 右臂 CAN 接口名 (真实硬件用)
        "arm_prefix": ""                    # 前缀为空
    }

    # ==========================================
    # 2. 解析 URDF 和 SRDF
    # ==========================================
    # 构建 xacro 文件的完整路径
    xacro_path = os.path.join(get_package_share_directory(DESCRIPTION_PKG), "urdf", "robot", URDF_FILE_NAME)
    # 处理 xacro 文件，传入参数 (mappings)，生成 XML 对象
    doc = xacro.process_file(xacro_path, mappings=XACRO_ARGS)
    # 将 XML 转换为字符串，封装在字典中，准备传给 Node 参数
    robot_description = {"robot_description": doc.toprettyxml(indent="  ")}

    # 构建 SRDF 文件的完整路径
    srdf_path = os.path.join(get_package_share_directory(MOVEIT_CONFIG_PKG), "config", SRDF_FILE_NAME)
    # 处理 SRDF 文件
    srdf_doc = xacro.process_file(srdf_path) 
    # 封装 SRDF 字符串
    robot_description_semantic = {"robot_description_semantic": srdf_doc.toprettyxml(indent="  ")}

    # ==========================================
    # 3. 加载配置文件 (MoveIt & ROS2 Control)
    # ==========================================
    
    # [A] Kinematics (运动学求解器配置，如 KDL 或 IKFast)
    kinematics_yaml = load_yaml(MOVEIT_CONFIG_PKG, "config/kinematics.yaml")
    
    # [B] Joint Limits (关节速度、加速度限位)
    joint_limits_yaml = load_yaml(MOVEIT_CONFIG_PKG, "config/joint_limits.yaml")
    # 确保格式正确，必须包含 'joint_limits' 键
    if joint_limits_yaml and 'joint_limits' not in joint_limits_yaml:
        joint_limits_yaml = {'joint_limits': joint_limits_yaml}
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    # [C] OMPL Planning (路径规划算法配置)
    # 这里的 request_adapters 列表非常关键，用于处理起始点、轨迹平滑等
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"], # 指定使用 OMPL 管道
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            # 定义请求适配器链：处理坐标系 -> 验证边界 -> 检查起始状态 -> 填充请求 -> *时间参数化*
            # AddTimeOptimalParameterization 用于给生成的几何路径加上时间戳(速度/加速度)
            # [修改后的代码]
            "request_adapters": """default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints default_planner_request_adapters/AddTimeOptimalParameterization""",
            "start_state_max_bounds_error": 0.1, # 允许起始状态有一定的误差
            "planner_configs": {
                # 默认规划算法 RRTConnect
                "RRTConnectkConfigDefault": {"type": "geometric::RRTConnect", "range": 0.0},
            }
        }
    }

    # [D] MoveIt Controllers (MoveIt 端如何与 ros2_control 通讯)
    moveit_controllers_yaml = load_yaml(MOVEIT_CONFIG_PKG, "config/moveit_controllers.yaml")
    trajectory_execution = moveit_controllers_yaml
    # 指定 MoveIt 控制器管理器插件
    trajectory_execution['moveit_controller_manager'] = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    # 执行时间缩放因子 (允许比计划慢一点)
    trajectory_execution['allowed_execution_duration_scaling'] = 1.2
    # 目标时间容差
    trajectory_execution['allowed_goal_duration_margin'] = 0.5
    # 起始位置容差 (如果在容差内，MoveIt 会认为已经在起点了)
    trajectory_execution['allowed_start_tolerance'] = 0.05

    # [E] ROS2 Controllers Config (底层控制器参数文件路径)
    # 必须传递文件路径，以便 ros2_control_node 能够读取完整的参数树
    ros2_controllers_path = os.path.join(
        get_package_share_directory(BRINGUP_PKG),
        "config", "v10_controllers", CONTROLLERS_YAML_NAME
    )
    
    # [E-Extra] 参数覆盖字典 (Runtime Override)
    # 这是一个重要的补丁：强制开启 open_loop_control (开环控制)。
    # 在使用 Fake Hardware 时，如果不开启此项，控制器可能会因为没有收到正确的状态反馈而拒绝执行目标 ("Goal rejected")。
    ros2_controllers_override = {
        "left_joint_trajectory_controller": {
            "ros__parameters": {
                "open_loop_control": True, # 忽略状态反馈误差
                "constraints": {
                    "stopped_velocity_tolerance": 0.01,
                    "goal_time": 0.5 # 放宽目标到达时间限制
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

    # [F] Monitor (MoveIt 场景监视器配置)
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,     # 发布完整的规划场景
        'publish_geometry_updates': True,   # 发布几何体更新 (如障碍物)
        'publish_state_updates': True,      # 发布机器人状态更新
        'publish_transforms_updates': True, # 发布 TF 更新
    }

    # ==========================================
    # 4. 定义节点 (Nodes)
    # ==========================================

    # [Node 1] Robot State Publisher
    # 作用：读取 URDF，并根据关节状态发布机器人的 TF 树
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description], # 传入 URDF 内容
    )

    # [Node 2] ROS2 Control Node (控制器管理器)
    # 作用：加载硬件接口，管理和调度控制器
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,         # 需要 URDF 来解析 ros2_control 标签
            ros2_controllers_path,     # 加载原始配置文件 (定义了 type, joints 等)
            ros2_controllers_override  # 加载覆盖参数 (强制覆盖 open_loop_control)
        ],
    )

    # [Node 3] Move Group (MoveIt 主节点)
    # 作用：提供路径规划、逆运动学求解、碰撞检测等核心功能
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,             # URDF
            robot_description_semantic,    # SRDF
            {"robot_description_kinematics": kinematics_yaml}, # IK 配置
            robot_description_planning,    # 关节限位
            ompl_planning_pipeline_config, # OMPL 规划管道
            trajectory_execution,          # 轨迹执行配置
            planning_scene_monitor_parameters, # 监视器配置
            {"use_sim_time": False},       # 使用系统时间 (非仿真时间)
        ],
    )

    # [Node 4] RViz (可视化界面)
    # 作用：显示机器人模型、交互式 Marker、规划路径
    rviz_config_file = os.path.join(get_package_share_directory(MOVEIT_CONFIG_PKG), "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file], # 加载指定的 .rviz 配置
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config, 
            {"robot_description_kinematics": kinematics_yaml},
        ],
    )

    # ==========================================
    # 5. Spawners (控制器加载器)
    # ==========================================
    
    # 加载关节状态广播器 (读取硬件位置 -> 发布 /joint_states)
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # 加载手臂轨迹控制器 (负责执行 MoveIt 发出的轨迹)
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_joint_trajectory_controller", "right_joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # 加载夹爪控制器
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gripper_controller", "right_gripper_controller", "-c", "/controller_manager"],
    )

    # ==========================================
    # 6. 返回 Launch Description
    # ==========================================
    return LaunchDescription([
        # 1. 首先启动状态发布器和控制器管理器
        robot_state_publisher_node,
        ros2_control_node,
        
        # 2. 延时 1.5秒 等待管理器就绪后，加载 Joint State Broadcaster
        # 必须先有 JSB，MoveIt 才能获取当前机械臂姿态
        TimerAction(period=1.5, actions=[jsb_spawner]),
        
        # 3. 再延时到 3.5秒，加载手臂控制器
        # 确保 JSB 已经运行，避免依赖冲突
        TimerAction(period=3.5, actions=[arm_spawner]),
        
        # 4. 最后加载夹爪控制器
        TimerAction(period=4.5, actions=[gripper_spawner]),
        
        # 5. 启动 Move Group (核心规划层)
        run_move_group_node,
        
        # 6. 启动 RViz 可视化
        rviz_node
    ])