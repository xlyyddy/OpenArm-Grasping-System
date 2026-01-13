# OpenArm 视觉引导抓取系统

## 项目简介

本课程设计基于ROS2平台，实现了OpenArm双臂机器人的视觉引导抓取系统。系统采用MuJoCo物理仿真环境进行验证，通过集成深度相机获取RGB和深度图像，基于YOLOv8深度学习模型实现目标检测，并使用MoveIt Task Constructor (MTC)进行抓取规划。在实现过程中，首先搭建了OpenArm双臂机器人URDF模型，在MuJoCo中集成了相机和环境模型（桌子、苹果、香蕉），开发了YOLO检测节点实现目标定位，并开发了MTC抓取规划节点实现完整抓取流程。系统实现了从像素坐标到世界坐标的精确转换，通过直接获取MuJoCo相机外参的方法简化了坐标转换流程。仿真实验验证了系统的有效性，目标检测精度和抓取成功率达到了预期目标。本系统为机器人视觉抓取任务提供了一个完整的仿真验证平台，为后续实物验证奠定了基础。

## 关键词

ROS2；OpenArm；YOLO目标检测；MuJoCo仿真；MoveIt Task Constructor

## 项目结构

```
src/
├── joint_state_pkg/              # 关节状态订阅包
├── moveit_task_constructor/      # MoveIt Task Constructor核心库
├── mtc_tutorial/                 # MTC教程示例
├── ompl/                         # OMPL运动规划库
├── openarm_cam_pkg/              # OpenArm相机包
├── openarm_can/                  # OpenArm CAN通信库
├── openarm_description/         # OpenArm机器人描述文件（URDF）
├── openarm_joint_control/        # OpenArm关节控制包
├── openarm_moveit_config/        # OpenArm MoveIt配置
├── openarm_mujoco/               # OpenArm MuJoCo仿真环境
├── py_binding_tools/             # Python绑定工具
└── yolo_target_detection/        # YOLO目标检测包
```

## 主要功能

* **机器人模型**：完整的OpenArm双臂机器人URDF模型
* **视觉感知**：基于YOLOv8的目标检测与定位
* **深度相机**：RGB和深度图像获取
* **运动规划**：基于MoveIt Task Constructor的抓取规划
* **物理仿真**：MuJoCo仿真环境验证

## 环境要求

* ROS2 (Humble/Galactic)
* MuJoCo
* Python 3.x
* MoveIt 2

## 安装与构建

1. 克隆仓库

```bash
git clone https://github.com/xlyyddy/OpenArm-Grasping-System.git
cd OpenArm-Grasping-System
```

2. 构建工作空间

```bash
colcon build
source install/setup.bash
```

## 使用方法

### 1. 启动rviz可视化窗口

```bash
ros2 launch mtc_tutorial mtc_demo.launch.py
```

### 2. 启动mujoco窗口

```bash
ros2 run joint_state_pkg joint_state_subscriber
```

### 3. 获取物体的坐标和rgb图片(深度图片)

```bash
ros2 run yolo_target_detection yolo_target_detection
```

### 4. 启动抓取程序

```bash
ros2 launch mtc_tutorial pick_place_demo.launch.py
```

## 许可证

本项目遵循相应的开源许可证。

## 贡献

欢迎提交Issue和Pull Request。

