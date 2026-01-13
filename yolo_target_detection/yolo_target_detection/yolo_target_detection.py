#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import mujoco
import glfw
import os
import time
from scipy.spatial.transform import Rotation as R


class MuJoCoYoloGraspingNode(Node):
    def __init__(self):
        super().__init__('mujoco_yolo_grasping_node')
        
        # 深度校准因子
        self.CALIBRATION_FACTOR = 0.8515

        # 深度可视化参数
        self.FIXED_MIN_DEPTH = 0.1  # 固定最小深度（相机近平面）
        self.FIXED_MAX_DEPTH = 2.0  # 固定最大显示深度
        
        # 初始化YOLO模型
        self.model = YOLO("yolov8n.pt")  # 可以替换为你训练的模型
        self.bridge = CvBridge()
        
        # 获取一次图像
        self.rgb_image, self.depth_image, self.camera_params = self.get_mujoco_images_once()
        
        if self.rgb_image is None or self.depth_image is None:
            self.get_logger().error("无法从MuJoCo获取图像")
            return
        
        # 相机参数
        self.image_width = 640
        self.image_height = 480
        
        # 从camera_params获取实际相机参数
        znear, zfar, fovy, camera_pos, camera_rot_mat = self.camera_params
        
        # 计算内参矩阵
        self.fx = self.fy = (self.image_height / 2) / np.tan(np.radians(fovy / 2))
        self.cx = self.image_width / 2
        self.cy = self.image_height / 2
        
        # 存储相机位姿用于坐标转换
        self.camera_pos = camera_pos
        self.camera_rot_mat = camera_rot_mat
        
        # 创建发布者
        self.rgb_pub = self.create_publisher(Image, '/mujoco/rgb_image', 10)
        self.depth_pub = self.create_publisher(Image, '/mujoco/depth_image', 10)
        self.obb_pub = self.create_publisher(Image, '/yolo/detection_image', 10)
        self.camera_coord_pub = self.create_publisher(PointStamped, '/yolo/object_camera_coordinates', 10)
        self.world_coord_pub = self.create_publisher(PoseStamped, '/yolo/object_world_pose', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/mujoco/camera_info', 10)
        
        # 目标类别
        self.target_class = "sports ball"  # 可修改为其他类别
        self.class_names = self.model.names
        
        # 发布相机内参
        self.publish_camera_info(fovy)
        
        # 启动检测
        self.detection_timer = self.create_timer(0.1, self.run_detection)  # 每0.1秒检测一次
        
        self.get_logger().info("MuJoCo YOLO抓取节点已启动")
    
    def get_calibrated_depth_conversion(self, calibration_factor=0.8515):
        """获取校准的深度转换函数"""
        CALIBRATION_FACTOR = calibration_factor
        
        def calibrated_depth_buffer_to_linear_depth(depth_buffer, near=0.1, far=100.0):
            """校准的非线性深度缓冲区转换"""
            z_ndc = 2.0 * depth_buffer - 1.0
            linear_depth = 2.0 * near * far / (far + near - z_ndc * (far - near))
            return linear_depth * CALIBRATION_FACTOR
        
        return calibrated_depth_buffer_to_linear_depth
    
    def pixel_to_camera_coords_corrected(self, pixel_x, pixel_y, depth_value, 
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
        return np.array([x_camera, -y_camera, -z_linear])
    
    def camera_to_world_coords_corrected(self, point_camera, camera_pos, camera_rot_mat):
        """修正的相机坐标到世界坐标转换"""
        # 使用标准变换：P_world = T + R * P_camera
        return camera_pos + camera_rot_mat @ point_camera
    
    def get_mujoco_images_once(self):
        """从MuJoCo获取一次RGB和深度图像"""
        try:
            # 1. 加载模型
            scene_path = "/home/ubuntuhjx/openarm_ws/src/openarm_mujoco/v1/scene_withcamera.xml"
            
            if not os.path.exists(scene_path):
                self.get_logger().error(f"找不到场景文件: {scene_path}")
                return None, None, None
            
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
            
            # 创建上下文
            try:
                context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
            except (TypeError, AttributeError):
                context = mujoco.MjrContext(model, 150)
            
            # 4. 配置相机
            camera_name = "rgb_camera"
            camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            
            if camera_id == -1:
                self.get_logger().error(f"未找到相机: {camera_name}")
                return None, None, None
            
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
            camera.fixedcamid = camera_id
            
            # 设置视口
            viewport = mujoco.MjrRect(0, 0, resolution[0], resolution[1])
            
            # 获取相机参数
            znear = model.vis.map.znear
            zfar = model.vis.map.zfar
            fovy = model.cam_fovy[camera_id]
            
            camera_data = data.cam(camera_id)
            camera_pos = camera_data.xpos
            camera_rot_mat = camera_data.xmat.reshape(3, 3)
            
            self.get_logger().info(f"获取相机参数: znear={znear:.3f}, zfar={zfar:.3f}, fovy={fovy:.1f}")
            
            width, height = resolution
            
            # 获取校准的深度转换函数
            depth_converter = self.get_calibrated_depth_conversion(self.CALIBRATION_FACTOR)
            
            # 执行一步物理仿真
            mujoco.mj_step(model, data)
            
            # 更新场景并渲染
            mujoco.mjv_updateScene(model, data, mujoco.MjvOption(), None, camera, 
                                  mujoco.mjtCatBit.mjCAT_ALL, scene)
            mujoco.mjr_render(viewport, scene, context)
            
            # 读取像素数据
            rgb_buffer = np.zeros((height, width, 3), dtype=np.uint8)
            depth_buffer = np.zeros((height, width), dtype=np.float32)
            mujoco.mjr_readPixels(rgb_buffer, depth_buffer, viewport, context)
            
            # 翻转图像
            rgb_image = np.flipud(rgb_buffer)
            depth_buffer = np.flipud(depth_buffer)
            
            # 校准深度
            linear_depth = depth_converter(depth_buffer, znear, zfar)
            
            # 清理资源
            glfw.terminate()
            
            self.get_logger().info("成功从MuJoCo获取图像")
            
            # 返回RGB图像、深度图像和相机参数
            return rgb_image, linear_depth, (znear, zfar, fovy, camera_pos, camera_rot_mat)
            
        except Exception as e:
            self.get_logger().error(f"从MuJoCo获取图像失败: {e}")
            return None, None, None
    
    def publish_camera_info(self, fovy):
        """发布相机内参"""
        camera_info = CameraInfo()
        camera_info.header = Header()
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.header.frame_id = "camera_frame"
        
        camera_info.width = self.image_width
        camera_info.height = self.image_height
        
        # 计算内参
        fx = fy = (self.image_height / 2) / np.tan(np.radians(fovy / 2))
        cx = self.image_width / 2
        cy = self.image_height / 2
        
        # 内参矩阵 (3x3 row-major)
        camera_info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]
        
        # 畸变参数 (默认无畸变)
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 投影矩阵 (3x4 row-major)
        camera_info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        self.camera_info_pub.publish(camera_info)
        self.get_logger().info("已发布相机内参")
    
    def run_detection(self):
        """运行YOLO检测并发布结果"""
        if self.rgb_image is None or self.depth_image is None:
            self.get_logger().warn("没有可用的图像进行检测")
            return
        
        try:
            # 发布原始RGB图像
            rgb_msg = self.bridge.cv2_to_imgmsg(
                cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR), 
                encoding="bgr8"
            )
            rgb_msg.header = Header()
            rgb_msg.header.stamp = self.get_clock().now().to_msg()
            rgb_msg.header.frame_id = "camera_frame"
            self.rgb_pub.publish(rgb_msg)
            
            depth_normalized = np.clip(
                (self.depth_image - self.FIXED_MIN_DEPTH) / (self.FIXED_MAX_DEPTH - self.FIXED_MIN_DEPTH), 
                0, 1
            )
            depth_normalized_8bit = (depth_normalized * 255).astype(np.uint8)

            # 发布深度图像（转换为16位，单位：毫米）
            depth_msg = self.bridge.cv2_to_imgmsg(depth_normalized_8bit, encoding="mono8")
            depth_msg.header = Header()
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = "camera_frame"
            self.depth_pub.publish(depth_msg)
            
            # 运行YOLO检测
            results = self.model(self.rgb_image)
            
            if len(results) > 0:
                result = results[0]
                boxes = result.boxes
                
                if boxes is not None and len(boxes) > 0:
                    # 在图像上绘制边界框
                    result_image = result.plot()
                    
                    # 处理每个检测结果
                    for i, box in enumerate(boxes):
                        class_id = int(box.cls[0])
                        class_name = self.class_names[class_id]
                        confidence = float(box.conf[0])
                        
                        # 只处理目标类别
                        if class_name != self.target_class:
                            continue
                        
                        # 获取边界框坐标
                        x_center, y_center, width, height = box.xywh[0]
                        
                        # 转换为整数像素坐标
                        x_center_int = int(x_center)
                        y_center_int = int(y_center)
                        
                        # 获取深度值
                        if (0 <= y_center_int < self.depth_image.shape[0] and 
                            0 <= x_center_int < self.depth_image.shape[1]):
                            
                            depth = self.depth_image[y_center_int, x_center_int]
                            
                            if depth > 0 and not np.isnan(depth) and not np.isinf(depth):
                                # 计算相机坐标系下的3D坐标
                                camera_point = self.convert_pixel_to_camera_coordinates(
                                    x_center_int, y_center_int, depth
                                )
                                
                                if camera_point is not None:
                                    # 发布相机坐标系下的坐标
                                    camera_point_msg = PointStamped()
                                    camera_point_msg.header = Header()
                                    camera_point_msg.header.stamp = self.get_clock().now().to_msg()
                                    camera_point_msg.header.frame_id = "camera_frame"
                                    camera_point_msg.point.x = float(camera_point[0])
                                    camera_point_msg.point.y = float(camera_point[1])
                                    camera_point_msg.point.z = float(camera_point[2])
                                    self.camera_coord_pub.publish(camera_point_msg)
                                    
                                    # 计算世界坐标系下的坐标
                                    znear, zfar, fovy, camera_pos, camera_rot_mat = self.camera_params
                                    
                                    # 使用MuJoCo深度缓冲值（需要从深度图反向计算）
                                    depth_value = self.get_depth_value_from_linear_depth(depth, znear, zfar)
                                    
                                    point_camera = self.pixel_to_camera_coords_corrected(
                                        x_center_int, y_center_int, depth_value,
                                        self.image_width, self.image_height, fovy, 
                                        znear, zfar, self.CALIBRATION_FACTOR
                                    )
                                    
                                    point_world = self.camera_to_world_coords_corrected(
                                        point_camera, camera_pos, camera_rot_mat
                                    )
                                    
                                    # 发布世界坐标系下的位姿
                                    world_pose_msg = PoseStamped()
                                    world_pose_msg.header = Header()
                                    world_pose_msg.header.stamp = self.get_clock().now().to_msg()
                                    world_pose_msg.header.frame_id = "world"
                                    world_pose_msg.pose.position.x = float(point_world[0])
                                    world_pose_msg.pose.position.y = float(point_world[1]) - 0.019
                                    world_pose_msg.pose.position.z = float(point_world[2]) - 0.059
                                    world_pose_msg.pose.orientation.w = 1.0
                                    self.world_coord_pub.publish(world_pose_msg)
                                    
                                    self.get_logger().info(
                                        f"检测到 {class_name} (置信度: {confidence:.2f}):\n"
                                        f"  像素坐标: ({x_center_int}, {y_center_int})\n"
                                        f"  深度: {depth:.3f}米\n"
                                        f"  相机坐标: ({camera_point[0]:.3f}, {camera_point[1]:.3f}, {camera_point[2]:.3f})\n"
                                        f"  世界坐标: ({point_world[0]:.3f}, {world_pose_msg.pose.position.y:.3f}, {world_pose_msg.pose.position.z:.3f})"
                                    )
                                    
                                    # 在图像上显示3D坐标
                                    label_3d = f"({camera_point[0]:.2f}, {camera_point[1]:.2f}, {camera_point[2]:.2f})m"
                                    cv2.putText(
                                        result_image, 
                                        label_3d, 
                                        (int(x_center - width/2), int(y_center - height/2) - 30),
                                        cv2.FONT_HERSHEY_SIMPLEX, 
                                        0.5, 
                                        (0, 255, 255), 
                                        2
                                    )
                    
                    # 发布带有检测框的图像
                    try:
                        obb_image_msg = self.bridge.cv2_to_imgmsg(
                            cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR), 
                            encoding="bgr8"
                        )
                        obb_image_msg.header = Header()
                        obb_image_msg.header.stamp = self.get_clock().now().to_msg()
                        obb_image_msg.header.frame_id = "camera_frame"
                        self.obb_pub.publish(obb_image_msg)
                    except Exception as e:
                        self.get_logger().error(f"发布检测图像失败: {e}")
        
        except Exception as e:
            self.get_logger().error(f"YOLO检测失败: {e}")
    
    def get_depth_value_from_linear_depth(self, linear_depth, near=0.1, far=100.0):
        """从线性深度计算深度缓冲值"""
        # 反向计算：线性深度 = 2.0 * near * far / (far + near - z_ndc * (far - near))
        # 其中 z_ndc = 2.0 * depth_buffer - 1.0
        # 先反转校准
        linear_depth_uncalibrated = linear_depth / self.CALIBRATION_FACTOR
        
        # 计算z_ndc
        z_ndc = (far + near - 2.0 * near * far / linear_depth_uncalibrated) / (far - near)
        
        # 计算深度缓冲值
        depth_buffer = (z_ndc + 1.0) / 2.0
        
        return depth_buffer
    
    def convert_pixel_to_camera_coordinates(self, u, v, depth):
        """将像素坐标转换为相机坐标系下的3D坐标"""
        if depth <= 0 or np.isnan(depth) or np.isinf(depth):
            return None
        
        # 相机坐标系下的坐标
        camera_x = (u - self.cx) * depth / self.fx
        camera_y = (v - self.cy) * depth / self.fy
        camera_z = depth
        
        return np.array([camera_x, camera_y, camera_z])


def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoYoloGraspingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()