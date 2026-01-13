# Copyright 2025 Enactic, Inc.
#
# 基于Apache许可证2.0版（以下简称"许可证"）授权；
# 除非遵守许可证，否则您不得使用此文件。
# 您可以在以下位置获取许可证的副本：
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# 除非适用法律要求或书面同意，否则根据许可证分发的软件
# 按"原样"分发，不附带任何明示或暗示的担保或条件。
# 请参阅许可证以了解管理权限和限制的具体语言。

import os
import xacro  # 用于处理Xacro文件的库

from ament_index_python.packages import get_package_share_directory  # 获取ROS包路径的工具

from launch import LaunchDescription, LaunchContext  # 启动描述和上下文类
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # 启动参数声明和自定义函数动作
from launch.substitutions import LaunchConfiguration  # 启动配置参数替换类

from launch_ros.actions import Node  # ROS节点启动动作类


def robot_state_publisher_spawner(context: LaunchContext, arm_type, ee_type, bimanual):
    """
    生成并启动robot_state_publisher节点的函数
    用于发布机器人模型的状态信息（从URDF文件解析）
    """
    # 将启动配置参数转换为字符串
    arm_type_str = context.perform_substitution(arm_type)
    ee_type_str = context.perform_substitution(ee_type)
    bimanual_str = context.perform_substitution(bimanual)

    # 构建Xacro文件的路径
    xacro_path = os.path.join(
        get_package_share_directory("openarm_description"),  # 获取openarm_description包的路径
        "urdf", "robot", f"{arm_type_str}.urdf.xacro"  # 拼接URDF文件路径（如v10.urdf.xacro）
    )

    # 处理Xacro文件，生成URDF描述字符串
    robot_description = xacro.process_file(
        xacro_path,
        mappings={  # 传递Xacro文件中需要的参数
            "arm_type": arm_type_str,
            "ee_type": ee_type_str,
            "bimanual": bimanual_str,
        }
    ).toprettyxml(indent="  ")  # 格式化XML输出

    # 返回包含robot_state_publisher节点的列表
    return [
        Node(
            package="robot_state_publisher",  # 节点所属的包
            executable="robot_state_publisher",  # 可执行文件名
            name="robot_state_publisher",  # 节点名称
            output="screen",  # 输出到屏幕
            parameters=[{"robot_description": robot_description}],  # 传递机器人描述参数
        )
    ]


def rviz_spawner(context: LaunchContext, bimanual):
    """
    生成并启动RViz2节点的函数
    根据是否为双机械臂配置加载不同的RViz配置文件
    """
    # 将双机械臂配置参数转换为字符串
    bimanual_str = context.perform_substitution(bimanual)

    # 根据双机械臂配置选择对应的RViz配置文件
    rviz_config_file = "bimanual.rviz" if bimanual_str.lower() == "true" else "arm_only.rviz"
    # 构建RViz配置文件的路径
    rviz_config_path = os.path.join(
        get_package_share_directory("openarm_description"),
        "rviz", rviz_config_file  # 拼接RViz配置文件路径
    )

    # 返回包含RViz2节点的列表
    return [
        Node(
            package="rviz2",  # 节点所属的包
            executable="rviz2",  # 可执行文件名
            name="rviz2",  # 节点名称
            arguments=["--display-config", rviz_config_path],  # 指定加载的配置文件
            output="screen"  # 输出到屏幕
        ),
    ]


def generate_launch_description():
    """生成启动描述，定义所有需要启动的节点和参数"""
    # 声明机械臂类型参数（必填，如v10）
    arm_type_arg = DeclareLaunchArgument(
        "arm_type",
        description="要可视化的机械臂类型（例如：v10）"
    )

    # 声明末端执行器类型参数（默认值为openarm_hand，可选none）
    ee_type_arg = DeclareLaunchArgument(
        "ee_type",
        default_value="openarm_hand",
        description="要附加的末端执行器类型（例如：openarm_hand或none）"
    )

    # 声明是否使用双机械臂配置的参数（默认值为false）
    bimanual_arg = DeclareLaunchArgument(
        "bimanual",
        default_value="false",
        description="是否使用双机械臂配置"
    )

    # 创建启动配置参数对象，用于在函数间传递参数
    arm_type = LaunchConfiguration("arm_type")
    ee_type = LaunchConfiguration("ee_type")
    bimanual = LaunchConfiguration("bimanual")

    # 创建加载robot_state_publisher节点的自定义函数动作
    robot_state_publisher_loader = OpaqueFunction(
        function=robot_state_publisher_spawner,
        args=[arm_type, ee_type, bimanual]  # 传递所需参数
    )

    # 创建加载RViz节点的自定义函数动作
    rviz_loader = OpaqueFunction(
        function=rviz_spawner,
        args=[bimanual]  # 传递双机械臂配置参数
    )

    # 返回启动描述对象，包含所有参数声明和节点
    return LaunchDescription([
        arm_type_arg,
        ee_type_arg,
        bimanual_arg,
        robot_state_publisher_loader,  # 加载机器人状态发布器节点
        Node(  # 启动关节状态发布器的图形界面节点
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui"
        ),
        rviz_loader,  # 加载RViz节点
    ])