 import mujoco
import mujoco.viewer
import numpy as np
import glfw
import cv2
import os
import threading
import time


def main():
    # 1. 加载模型（必须在创建任何上下文之前）
    script_dir = os.path.dirname(os.path.abspath(__file__))
    scene_path = os.path.join(script_dir, "v1", "scene_withcamera.xml")
    
    if not os.path.exists(scene_path):
        raise FileNotFoundError(scene_path)
    
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)
    
    # 2. 初始化离屏渲染上下文（在单独的OpenGL上下文中）
    resolution = (640, 480)
    glfw.init()
    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)  # 关键：设置为不可见以实现离屏
    window = glfw.create_window(resolution[0], resolution[1], "Offscreen", None, None)
    glfw.make_context_current(window)

    # 3. 设置场景和上下文
    scene = mujoco.MjvScene(model, maxgeom=10000)
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)

    # 4. 配置相机 (通过名称获取在XML中定义的相机)
    camera_name = "rgb_camera"  # 确保此名称与XML文件中的相机定义一致
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    
    if camera_id == -1:
        raise RuntimeError(f"未找到相机: {camera_name}")
    
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FIXED  # 或 mjCAMERA_TRACKING
    camera.fixedcamid = camera_id

    # 设置视口
    viewport = mujoco.MjrRect(0, 0, resolution[0], resolution[1])

    # 用于控制循环的全局变量
    running = True
    viewer_sync = None  # 用于viewer同步的对象

    print("=" * 60)
    print("重要提示：")
    print("1. MuJoCo GUI窗口和离屏渲染共享同一个仿真状态")
    print("2. 在GUI中手动操作机械臂时，离屏渲染会实时同步显示")
    print("3. 按 ESC 退出（在OpenCV窗口中）")
    print("=" * 60)

    # 启动MuJoCo交互式viewer（在单独线程中运行，共享同一个model和data）
    # 这样GUI和离屏渲染就共享同一个仿真状态了
    def run_viewer():
        global viewer_sync
        try:
            # MuJoCo 3.0+ 使用 launch_passive（被动模式）
            # launch_passive 是被动的，不会自动执行mj_step，只是显示当前状态
            # 用户在GUI中调节控制滑块时，会修改 data.ctrl
            # 主循环中的 mj_step 会读取 data.ctrl 并应用控制，更新 data.qpos
            with mujoco.viewer.launch_passive(model, data) as viewer:
                viewer_sync = viewer
                while running:
                    # viewer会自动同步显示data的当前状态
                    # 用户在GUI中的操作会修改data.ctrl（控制输入）
                    time.sleep(0.01)  # 避免占用太多CPU
        except AttributeError:
            # 如果launch_passive不存在，尝试其他方法
            print("警告：当前MuJoCo版本可能不支持launch_passive")
            print("将使用同步模式：离屏渲染会实时读取当前状态，但不自动步进")
        except Exception as e:
            print(f"无法启动viewer: {e}")
            print("将继续运行离屏渲染，但无法在GUI中手动操作")

    viewer_thread = threading.Thread(target=run_viewer, daemon=True)
    viewer_thread.start()
    
    # 等待viewer启动
    time.sleep(0.5)

    # 主循环：实时读取并渲染当前状态
    # 核心机制：每一步仿真后立即渲染（根据 md 文档）
    # 注意：launch_passive 的viewer是被动的，不会自动步进，需要我们在主循环中调用mj_step
    render_count = 0
    while running:
        # 关键：必须调用 mj_step 来应用控制输入（data.ctrl）并推进仿真
        # 用户在GUI中调节的控制值存储在 data.ctrl 中，mj_step 会读取并应用它们
        # mj_step 内部已经包含了 mj_forward，所以不需要单独调用
        mujoco.mj_step(model, data)
        
        # 实时渲染（不降采样，确保能实时看到GUI中的操作）
        render_count += 1
        if render_count % 2 == 0:  # 轻微降采样，每2帧渲染一次
            # 更新场景（捕捉最新状态）
            mujoco.mjv_updateScene(model, data, mujoco.MjvOption(), None, camera, mujoco.mjtCatBit.mjCAT_ALL, scene)
            mujoco.mjr_render(viewport, scene, context)
            
            # 读取像素数据（RGB和深度）
            rgb_buffer = np.zeros((resolution[1], resolution[0], 3), dtype=np.uint8)
            depth_buffer = np.zeros((resolution[1], resolution[0]), dtype=np.float32)
            
            # 读取RGB和深度缓冲区
            mujoco.mjr_readPixels(rgb_buffer, depth_buffer, viewport, context)
            
            # 处理深度图像（重要：原始深度值需要转换）
            # depth_buffer 中的值是线性深度值，通常在近裁剪面和远裁剪面之间
            # 可视化为灰度图以供显示
            if depth_buffer.max() > depth_buffer.min():
                depth_image = (depth_buffer - depth_buffer.min()) / (depth_buffer.max() - depth_buffer.min() + 1e-6)
                depth_image = (depth_image * 255).astype(np.uint8)
            else:
                depth_image = np.zeros((resolution[1], resolution[0]), dtype=np.uint8)
            
            # 显示图像
            cv2.imshow('RGB View', cv2.cvtColor(np.flipud(rgb_buffer), cv2.COLOR_RGB2BGR))
            cv2.imshow('Depth View', np.flipud(depth_image))
        
        # 检查退出条件
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # 按ESC键退出
            running = False
            break

    cv2.destroyAllWindows()
    glfw.terminate()


if __name__ == "__main__":
    main()