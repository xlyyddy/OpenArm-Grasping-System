import mujoco
import mujoco.viewer
import numpy as np
import glfw
import cv2
import os
import time

def get_calibrated_depth_conversion():
    """获取校准的深度转换函数"""
    CALIBRATION_FACTOR = 0.8515
    
    def calibrated_depth_buffer_to_linear_depth(depth_buffer, near=0.1, far=100.0):
        """校准的非线性深度缓冲区转换"""
        z_ndc = 2.0 * depth_buffer - 1.0
        linear_depth = 2.0 * near * far / (far + near - z_ndc * (far - near))
        return linear_depth * CALIBRATION_FACTOR
    
    return calibrated_depth_buffer_to_linear_depth

def pixel_to_camera_coords_corrected(pixel_x, pixel_y, depth_value, 
                                     width=640, height=480, fovy=60.0, 
                                     near=0.1, far=100.0, calib_factor=1.0):
    """修正的像素坐标到相机坐标转换"""
    # 使用校准的深度转换
    z_ndc = 2.0 * depth_value - 1.0
    z_linear = 2.0 * near * far / (far + near - z_ndc * (far - near))
    z_linear *= calib_factor  # 应用校准
    
    # 相机内参
    f = height / (2.0 * np.tan(np.radians(fovy) / 2.0))
    cx = width / 2.0
    cy = height / 2.0
    
    # 像素坐标到相机坐标
    x_camera = (pixel_x - cx) * z_linear / f
    y_camera = (pixel_y - cy) * z_linear / f
    
    # 关键修正：Z分量取反
    # 因为在MuJoCo中，深度值对应的是从相机到物体的距离
    # 但相机坐标系可能是Z轴向内（相机看向正Z方向）
    return np.array([x_camera, -y_camera, -z_linear])

def camera_to_world_coords_corrected(point_camera, camera_pos, camera_rot_mat):
    """修正的相机坐标到世界坐标转换"""
    # 使用标准变换：P_world = T + R * P_camera
    return camera_pos + camera_rot_mat @ point_camera

def get_camera_params_from_model(model, data, camera_id):
    """从MuJoCo模型中获取相机参数"""
    fovy = model.cam_fovy[camera_id]
    znear = model.vis.map.znear
    zfar = model.vis.map.zfar
    
    camera_data = data.cam(camera_id)
    camera_pos = camera_data.xpos
    camera_rot_mat = camera_data.xmat.reshape(3, 3)
    
    return znear, zfar, fovy, camera_pos, camera_rot_mat

def main():
    # 1. 加载模型
    script_dir = os.path.dirname(os.path.abspath(__file__))
    scene_path = os.path.join(script_dir, "v1", "scene_withcamera.xml")
    
    if not os.path.exists(scene_path):
        raise FileNotFoundError(scene_path)
    
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)
    
    # 2. 初始化离屏渲染上下文
    resolution = (640, 480)
    glfw.init()
    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
    window = glfw.create_window(resolution[0], resolution[1], "Offscreen", None, None)
    glfw.make_context_current(window)

    # 3. 设置场景和上下文
    scene = mujoco.MjvScene(model, maxgeom=10000)
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)

    # 4. 配置相机
    camera_name = "rgb_camera"
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    
    if camera_id == -1:
        raise RuntimeError(f"未找到相机: {camera_name}")
    
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
    camera.fixedcamid = camera_id

    # 设置视口
    viewport = mujoco.MjrRect(0, 0, resolution[0], resolution[1])
    
    # 从模型中获取相机参数
    znear = model.vis.map.znear
    zfar = model.vis.map.zfar
    fovy = model.cam_fovy[camera_id]
    
    camera_data = data.cam(camera_id)
    camera_pos = camera_data.xpos
    camera_rot_mat = camera_data.xmat.reshape(3, 3)
    
    print(f"相机参数: znear={znear:.3f}, zfar={zfar:.3f}, fovy={fovy:.1f}")
    
    width, height = resolution
    
    # 获取校准的深度转换函数
    depth_converter = get_calibrated_depth_conversion()
    CALIBRATION_FACTOR = 0.8515
    
    # 深度可视化参数
    FIXED_MIN_DEPTH = 0.1  # 固定最小深度（相机近平面）
    FIXED_MAX_DEPTH = 2.0  # 固定最大显示深度
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.sync()
        
        frame_count = 0
        
        while viewer.is_running():
            step_start = time.time()
            
            viewer.sync()
            mujoco.mj_step(model, data)
            
            # 更新场景并渲染
            mujoco.mjv_updateScene(model, data, mujoco.MjvOption(), None, camera, mujoco.mjtCatBit.mjCAT_ALL, scene)
            mujoco.mjr_render(viewport, scene, context)
            
            # 读取像素数据
            rgb_buffer = np.zeros((height, width, 3), dtype=np.uint8)
            depth_buffer = np.zeros((height, width), dtype=np.float32)
            mujoco.mjr_readPixels(rgb_buffer, depth_buffer, viewport, context)
            
            # 使用校准的深度转换
            linear_depth = depth_converter(depth_buffer, znear, zfar)
            
            # 调试：打印深度统计信息
            if frame_count % 30 == 0:
                print(f"\n=== 帧 {frame_count} 深度统计 ===")
                print(f"深度缓冲区范围: min={depth_buffer.min():.6f}, max={depth_buffer.max():.6f}")
                print(f"线性深度范围: min={linear_depth.min():.4f}m, max={linear_depth.max():.4f}m")
                print(f"深度显示范围: {FIXED_MIN_DEPTH:.2f}-{FIXED_MAX_DEPTH:.2f}m")
            
            # 方案1：使用固定范围归一化（推荐）
            depth_normalized_fixed = np.clip(
                (linear_depth - FIXED_MIN_DEPTH) / (FIXED_MAX_DEPTH - FIXED_MIN_DEPTH), 
                0, 1
            )
            depth_image_fixed = (depth_normalized_fixed * 255).astype(np.uint8)
            
            # 显示图像
            cv2.imshow('RGB View', cv2.cvtColor(np.flipud(rgb_buffer), cv2.COLOR_RGB2BGR))
            cv2.imshow('Depth Fixed Range (0.1-2.0m)', np.flipud(depth_image_fixed))
            
            # 添加图例
            if frame_count % 30 == 0:
                print("\n深度图说明:")
                print("  1. Fixed Range: 使用固定范围 0.1-2.0m")
                print("  2. Log Scale: 使用对数刻度")
                print("  3. Dynamic Range: 使用动态范围（不推荐）")
            
            # 测试多个点
            if frame_count % 30 == 0:
                # 定义要测试的点
                test_points = [
                    (320, 240, "图像中心"),
                    (300, 200, "右上区域"),
                    (340, 280, "左下区域"),
                ]
                
                for px, py, label in test_points:
                    depth_value = depth_buffer[py, px]
                    
                    if depth_value > 0:
                        # 更新相机数据
                        camera_data = data.cam(camera_id)
                        camera_pos = camera_data.xpos
                        camera_rot_mat = camera_data.xmat.reshape(3, 3)
                        
                        # 使用修正的转换
                        point_camera = pixel_to_camera_coords_corrected(
                            px, py, depth_value,
                            width, height, fovy, znear, zfar, CALIBRATION_FACTOR
                        )
                        
                        point_world = camera_to_world_coords_corrected(
                            point_camera, camera_pos, camera_rot_mat
                        )
                        
                        print(f"\n{label} (帧 {frame_count}):")
                        print(f"  像素坐标: ({px}, {py})")
                        print(f"  深度值: {depth_value:.4f}")
                        print(f"  线性深度: {depth_converter(depth_value, znear, zfar):.3f}米")
                        print(f"  相机坐标: ({point_camera[0]:.3f}, {point_camera[1]:.3f}, {point_camera[2]:.3f})")
                        print(f"  世界坐标: ({point_world[0]:.3f}, {point_world[1]:.3f}, {point_world[2]:.3f})")
            
            frame_count += 1
            
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC键退出
                break
            elif key == ord('+'):  # 增加最大显示深度
                FIXED_MAX_DEPTH += 0.5
                print(f"最大显示深度增加到: {FIXED_MAX_DEPTH:.1f}米")
            elif key == ord('-'):  # 减少最大显示深度
                FIXED_MAX_DEPTH = max(0.5, FIXED_MAX_DEPTH - 0.5)
                print(f"最大显示深度减少到: {FIXED_MAX_DEPTH:.1f}米")
            elif key == ord('r'):  # 重置为默认值
                FIXED_MAX_DEPTH = 2.0
                print(f"重置最大显示深度为: {FIXED_MAX_DEPTH:.1f}米")
            
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    
    cv2.destroyAllWindows()
    glfw.terminate()

if __name__ == "__main__":
    main()
