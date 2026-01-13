# 深度相机使用说明

本文档说明如何运行 `depth_camera_example.py` 来使用 MuJoCo 深度相机功能。

## 环境要求

- Python 3.7 或更高版本
- Windows/Linux/macOS 操作系统
- 支持 OpenGL 的显卡驱动

## 依赖安装

### 1. 安装 MuJoCo

首先需要安装 MuJoCo Python 包：

```bash
pip install mujoco
```

或者从源码安装（推荐使用最新版本）：
```bash
pip install git+https://github.com/google-deepmind/mujoco.git
```

### 2. 安装其他依赖

```bash
pip install numpy opencv-python glfw
```

**注意**：在 Windows 上，`glfw` 可能需要额外的系统库。如果安装失败，可以尝试：

```bash
# Windows 上可能需要先安装 Visual C++ Redistributable
# 或者使用 conda 安装
conda install -c conda-forge glfw
```

在 Linux 上，可能需要安装系统库：
```bash
# Ubuntu/Debian
sudo apt-get install libglfw3-dev

# 或者使用 pip（如果系统库不可用）
pip install glfw
```

## 运行步骤

### 方法 1：直接运行 Python 脚本

1. 打开终端/命令提示符，进入项目目录：
   ```bash
   cd d:\code\ros2_ws\src\openarm_mujoco
   ```

2. 运行脚本：
   ```bash
   python depth_camera_example.py
   ```

   或者使用 Python 3：
   ```bash
   python3 depth_camera_example.py
   ```

### 方法 2：使用绝对路径运行

```bash
python d:\code\ros2_ws\src\openarm_mujoco\depth_camera_example.py
```

## 运行效果

运行成功后，你会看到：

1. **两个窗口**：
   - `MuJoCo RGB Camera`：显示 RGB 彩色图像
   - `MuJoCo Depth Camera`：显示深度图像（使用 JET 颜色映射，红色表示近，蓝色表示远）

2. **控制台输出**：
   - 显示相机 ID（如果找到相机）
   - 提示 "按 ESC 键退出..."

3. **退出程序**：
   - 在图像窗口激活状态下，按 **ESC 键**退出
   - 程序会自动保存最后一帧的 RGB 和深度图像到：
     - `debug_output_rgb.png`
     - `debug_output_depth.png`

## 文件结构

运行前确保以下文件存在：

```
src/openarm_mujoco/
├── depth_camera_example.py    # 主程序
├── v1/
│   ├── scene_withcamera.xml   # 带相机的场景文件
│   ├── openarm_bimanual.xml   # 机器人模型（被 scene_withcamera.xml 引用）
│   └── meshes/                # 模型网格文件
```

## 常见问题

### 1. 找不到 scene_withcamera.xml

**错误信息**：`FileNotFoundError: 场景文件未找到: ...`

**解决方法**：
- 确保 `v1/scene_withcamera.xml` 文件存在
- 检查文件路径是否正确

### 2. GLFW 初始化失败

**错误信息**：`glfw.init()` 失败或窗口创建失败

**解决方法**：
- 确保显卡驱动已正确安装
- 在 Linux 上，可能需要设置显示环境变量：
  ```bash
  export DISPLAY=:0
  ```
- 尝试更新显卡驱动

### 3. 找不到相机

**错误信息**：`camera_id` 为 -1 或不显示

**解决方法**：
- 检查 `scene_withcamera.xml` 中是否正确定义了 `rgb_camera`
- 确保相机名称与代码中的 `camera_name = "rgb_camera"` 一致

### 4. OpenCV 窗口无法显示

**错误信息**：窗口不显示或显示为空白

**解决方法**：
- 确保 OpenCV 正确安装：`pip install opencv-python`
- 在无图形界面的系统上，可能需要使用虚拟显示（Xvfb）

### 5. 模块导入错误

**错误信息**：`ModuleNotFoundError: No module named 'mujoco'`

**解决方法**：
```bash
pip install mujoco numpy opencv-python glfw
```

## 自定义配置

### 修改相机分辨率

在 `depth_camera_example.py` 中修改：
```python
resolution = (1280, 720)  # 改为你想要的分辨率
```

### 修改相机参数

在循环中修改相机跟踪参数：
```python
camera.distance = 1.5      # 相机与目标的距离
camera.azimuth = 45       # 水平方位角（度）
camera.elevation = -60    # 俯仰角（度）
```

### 使用固定相机（不跟踪）

取消注释并修改：
```python
camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
# 注释掉或删除跟踪相关的代码
```

### 修改跟踪目标

如果场景中有其他物体，可以修改跟踪的 body 名称：
```python
tracking_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "your_body_name")
```

## 技术说明

- **离屏渲染**：使用 GLFW 创建不可见的窗口进行离屏渲染，不显示 MuJoCo 主窗口
- **深度信息**：MuJoCo 返回的深度值在 [0, 1] 范围内，需要根据场景范围转换为实际距离
- **颜色映射**：深度图使用 OpenCV 的 JET 颜色映射进行可视化，便于观察深度变化

## 参考

- 知乎文章：https://zhuanlan.zhihu.com/p/1900713463807976784
- MuJoCo 官方文档：https://mujoco.readthedocs.io/
