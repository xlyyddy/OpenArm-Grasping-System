import mujoco
import numpy as np
import glfw
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import time
from scipy.spatial.transform import Rotation as R

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__("joint_state_subscriber")
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.callback,
            10
        )
        
        # 初始化关节位置列表
        self.left_positions = np.zeros(7)  # 左臂7个关节
        self.right_positions = np.zeros(7)  # 右臂7个关节
        self.left_finger_positions = np.zeros(2)  # 左手指2个关节
        self.right_finger_positions = np.zeros(2)  # 右手指2个关节
        
        # 用于存储关节顺序的列表
        self.openarm_left_joints = []
        self.openarm_right_joints = []
        self.openarm_left_finger_joints = []
        self.openarm_right_finger_joints = []
        
        # 用于标识是否已收到关节消息
        self.received_joint_states = False
        self.lock = threading.Lock()

    def callback(self, msg: JointState):
        with self.lock:
            # 首次收到消息时初始化关节名称列表
            if not self.received_joint_states:
                self.openarm_left_joints = [name for name in msg.name if "openarm_left_joint" in name]
                self.openarm_right_joints = [name for name in msg.name if "openarm_right_joint" in name]
                self.openarm_left_finger_joints = [name for name in msg.name if "openarm_left_finger_joint" in name]
                self.openarm_right_finger_joints = [name for name in msg.name if "openarm_right_finger_joint" in name]
                self.received_joint_states = True
            
            # 初始化位置数组
            temp_left = self.left_positions.copy()
            temp_right = self.right_positions.copy()
            temp_left_finger = self.left_finger_positions.copy()
            temp_right_finger = self.right_finger_positions.copy()
            
            # 提取关节位置
            for i, name in enumerate(msg.name):
                if "openarm_left_joint" in name:
                    # 提取关节编号 (假设关节名为 "openarm_left_joint1", "openarm_left_joint2", ...)
                    joint_num = int(name.split('openarm_left_joint')[1]) - 1
                    if 0 <= joint_num < 7:
                        temp_left[joint_num] = msg.position[i]
                elif "openarm_right_joint" in name:
                    joint_num = int(name.split('openarm_right_joint')[1]) - 1
                    if 0 <= joint_num < 7:
                        temp_right[joint_num] = msg.position[i]
                elif "openarm_left_finger_joint" in name:
                    joint_num = int(name.split('openarm_left_finger_joint')[1]) - 1
                    if 0 <= joint_num < 2:
                        temp_left_finger[joint_num] = msg.position[i]
                elif "openarm_right_finger_joint" in name:
                    joint_num = int(name.split('openarm_right_finger_joint')[1]) - 1
                    if 0 <= joint_num < 2:
                        temp_right_finger[joint_num] = msg.position[i]
            
            # 更新位置
            self.left_positions = np.array(temp_left)
            self.right_positions = np.array(temp_right)
            self.left_finger_positions = np.array(temp_left_finger)
            self.right_finger_positions = np.array(temp_right_finger)

class MuJoCoViewer:
    def __init__(self, model_path):
        # 加载模型
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # 启动viewer
        self.handle = mujoco.viewer.launch_passive(self.model, self.data)
        
        # 找到末端执行器的 body id
        self.left_end_effector_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, 'openarm_left_hand'
        )
        self.right_end_effector_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, 'openarm_right_hand'
        )
        
        # 找到苹果的body id
        self.apple_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, 'apple'
        )
        
        # 找到苹果关节的ID
        apple_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, 'apple_joint'
        )
        
        # 找到苹果在qpos中的索引
        if apple_joint_id != -1:
            # 自由关节在qpos中占用7个位置
            self.apple_qpos_start = self.model.jnt_qposadr[apple_joint_id]
            print(f"苹果在qpos中的起始索引: {self.apple_qpos_start}")
        else:
            # 如果没有找到关节，假设是18
            self.apple_qpos_start = 18
            print(f"未找到苹果关节，使用默认索引: {self.apple_qpos_start}")
        
        print(f"左手末端ID: {self.left_end_effector_id}")
        print(f"右手末端ID: {self.right_end_effector_id}")
        print(f"苹果物体ID: {self.apple_body_id}")
        
        if self.left_end_effector_id == -1 or self.right_end_effector_id == -1:
            print("警告: 未找到末端执行器")
        
        # 初始化ROS节点
        rclpy.init()
        self.joint_subscriber = JointStateSubscriber()
        
        # 固定力矩控制参数
        self.finger_close_torque = -10.0  # 夹爪闭合力矩
        self.finger_open_torque = 10.0  # 夹爪张开力矩
        
        # 阈值：夹爪目标位置小于此值时认为需要闭合
        self.close_threshold = 0.01
        
        # 运行状态
        self.running = True
        
        # 启动ROS线程
        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()
        
        # 调试计数器
        self.debug_counter = 0
        
        # 抓取状态
        self.right_grasping = False
        self.left_grasping = False
        
        # 存储抓取瞬间的手和苹果位置
        self.right_grasp_hand_pos = None
        self.right_grasp_apple_pos = None
        self.left_grasp_hand_pos = None
        self.left_grasp_apple_pos = None
        

    def ros_spin(self):
        """ROS后台处理线程"""
        while self.running and rclpy.ok():
            rclpy.spin_once(self.joint_subscriber, timeout_sec=0.001)

    def apply_joint_states(self):
        """应用从ROS接收到的关节状态"""
        with self.joint_subscriber.lock:
            if not self.joint_subscriber.received_joint_states:
                return
            
            # 获取目标位置
            target_left = self.joint_subscriber.left_positions
            target_right = self.joint_subscriber.right_positions
            target_left_finger = self.joint_subscriber.left_finger_positions
            target_right_finger = self.joint_subscriber.right_finger_positions
            
            # 1. 机械臂关节：直接位置控制
            self.data.qpos[:7] = target_left
            self.data.qpos[9:16] = target_right
            
            # 2. 夹爪关节：固定力矩控制
            # 左夹爪：根据目标位置决定力矩方向
            left_torque = np.zeros(2)
            for i in range(2):
                if target_left_finger[i] < self.close_threshold:
                    # 目标位置小，需要闭合
                    left_torque[i] = self.finger_close_torque
                else:
                    # 目标位置大，需要张开
                    left_torque[i] = self.finger_open_torque
            
            # 右夹爪：根据目标位置决定力矩方向
            right_torque = np.zeros(2)
            for i in range(2):
                if target_right_finger[i] < self.close_threshold:
                    # 目标位置小，需要闭合
                    right_torque[i] = self.finger_close_torque
                else:
                    # 目标位置大，需要张开
                    right_torque[i] = self.finger_open_torque
            
            # 检测碰撞并管理抓取
            self.manage_grasp(right_torque[0], left_torque[0])
            
            # 应用固定力矩
            self.data.ctrl[7:9] = left_torque
            self.data.ctrl[16:18] = right_torque
            
            # 如果正在抓取，更新苹果位置
            self.update_apple_position()
            
            self.debug_counter += 1
    
    def manage_grasp(self, right_torque, left_torque):
        """管理抓取：检测碰撞并添加/移除抓取"""
        # 检测所有碰撞
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            body1_id = self.model.geom_bodyid[contact.geom1]
            body2_id = self.model.geom_bodyid[contact.geom2]

            # 通过mj_id2name转换body_id为名称
            body1_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body1_id)
            body2_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body2_id)
            
            # 右手与苹果碰撞
            if(self.joint_subscriber.right_finger_positions[0] < 0.01):
                if (body1_name == "openarm_right_right_finger" or body2_name == "openarm_right_right_finger"):
                    if (body1_name == "apple" or body2_name == "apple"):
                        if right_torque == self.finger_close_torque and not self.right_grasping:
                            # 记录抓取瞬间的手和苹果位置
                            self.right_grasp_hand_pos = self.data.xpos[self.right_end_effector_id].copy()
                            self.right_grasp_apple_pos = self.data.xpos[self.apple_body_id].copy()
                            self.right_grasping = True
                            print(f"右手抓住苹果: 手={self.right_grasp_hand_pos}, 苹果={self.right_grasp_apple_pos}")
            
            # 左手与苹果碰撞
            if(self.joint_subscriber.left_finger_positions[0] < 0.1):
                if (body1_name == "openarm_left_right_finger" or body2_name == "openarm_left_right_finger"):
                    if (body1_name == "apple" or body2_name == "apple"):
                        if left_torque == self.finger_close_torque and not self.left_grasping:
                            # 记录抓取瞬间的手和苹果位置
                            self.left_grasp_hand_pos = self.data.xpos[self.left_end_effector_id].copy()
                            self.left_grasp_apple_pos = self.data.xpos[self.apple_body_id].copy()
                            self.left_grasping = True
                            print(f"左手抓住苹果: 手={self.left_grasp_hand_pos}, 苹果={self.left_grasp_apple_pos}")
        
        # 检查释放条件
        if self.right_grasping and self.joint_subscriber.right_finger_positions[0] >= 0.01:
            self.right_grasping = False
            self.right_grasp_hand_pos = None
            self.right_grasp_apple_pos = None
            print("右手释放苹果")
        
        if self.left_grasping and self.joint_subscriber.left_finger_positions[0] >= 0.01:
            self.left_grasping = False
            self.left_grasp_hand_pos = None
            self.left_grasp_apple_pos = None
            print("左手释放苹果")
    
    def update_apple_position(self):
        """更新苹果位置（相对位移法）"""
        if self.right_grasping and self.right_grasp_hand_pos is not None and self.right_grasp_apple_pos is not None:
            # 获取当前手的位置
            current_hand_pos = self.data.xpos[self.right_end_effector_id].copy()
            current_hand_quat = self.data.xquat[self.right_end_effector_id].copy()
            
            # 计算手部的位移
            hand_displacement = current_hand_pos - self.right_grasp_hand_pos
            
            # 计算新的苹果位置：抓取时的苹果位置 + 手部位移
            new_apple_pos = self.right_grasp_apple_pos + hand_displacement

            new_apple_quat = current_hand_quat.copy()  # 跟随手的姿态
            
            # 更新苹果的qpos
            self.data.qpos[self.apple_qpos_start:self.apple_qpos_start + 3] = new_apple_pos
            self.data.qpos[self.apple_qpos_start + 3:self.apple_qpos_start + 7] = [1, 0, 0, 0]
            
            # 更新手的位置和苹果位置用于下一次迭代
            self.right_grasp_hand_pos = current_hand_pos.copy()
            self.right_grasp_apple_pos = new_apple_pos.copy()
        
        elif self.left_grasping and self.left_grasp_hand_pos is not None and self.left_grasp_apple_pos is not None:
            # 获取当前手的位置
            current_hand_pos = self.data.xpos[self.left_end_effector_id].copy()
            
            # 计算手部的位移
            hand_displacement = current_hand_pos - self.left_grasp_hand_pos
            
            # 计算新的苹果位置：抓取时的苹果位置 + 手部位移
            new_apple_pos = self.left_grasp_apple_pos + hand_displacement
            
            # 更新苹果的qpos
            self.data.qpos[self.apple_qpos_start:self.apple_qpos_start + 3] = new_apple_pos
            
            # 更新手的位置和苹果位置用于下一次迭代
            self.left_grasp_hand_pos = current_hand_pos.copy()
            self.left_grasp_apple_pos = new_apple_pos.copy()
            
    
    def rotation_matrix_to_quaternion(self, rot_matrix):
        """将3x3旋转矩阵转换为四元数 [w, x, y, z]"""
        # 使用scipy转换
        rotation = R.from_matrix(rot_matrix)
        quat = rotation.as_quat()  # 返回 [x, y, z, w]
        
        # 转换为MuJoCo格式 [w, x, y, z]
        mujoco_quat = np.array([quat[3], quat[0], quat[1], quat[2]])
        return mujoco_quat

    def run_loop(self):
        """主运行循环"""
        try:
            # 设置相机视角
            self.handle.cam.distance = 3.0
            self.handle.cam.azimuth = 0.0
            self.handle.cam.elevation = -30.0
            self.handle.cam.lookat[:] = [0.0, 0.0, 0.0]
            
            # 主循环
            while self.handle.is_running() and self.running:
                # 更新模型前向动力学
                mujoco.mj_forward(self.model, self.data)
                
                # 应用关节状态
                self.apply_joint_states()
                
                # 执行一步物理模拟
                mujoco.mj_step(self.model, self.data)
                
                # 同步viewer
                self.handle.sync()
                
                # 控制循环频率
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            print("\n程序被用户中断")
        finally:
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        self.running = False
        if hasattr(self, 'handle'):
            self.handle.close()
        if rclpy.ok():
            rclpy.shutdown()
        print("程序已退出")

def main():
    # 模型路径
    model_path = '/home/ubuntuhjx/openarm_ws/src/openarm_mujoco/v1/scene_withcamera.xml'
    
    # 创建并运行viewer
    try:
        viewer = MuJoCoViewer(model_path)
        viewer.run_loop()
    except Exception as e:
        print(f"运行出错: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()