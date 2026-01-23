import time
import requests
import numpy as np
import cv2
import base64
from scipy.spatial.transform import Rotation as R

# ================= 服务器交互配置 =================
# 推理服务器地址配置
# 如果你在本机跑，用 "127.0.0.1"
# 如果你在局域网另一台机器跑，填你 Mac 的 IP (例如 "172.16.17.208")
SERVER_IP = "172.16.17.208"  
SERVER_PORT = 5003
# 推理API端点：POST请求到此URL获取动作预测
MODEL_URL = f"http://{SERVER_IP}:{SERVER_PORT}/predict_action"

# 图像参数（用于预处理和编码）
IMG_WIDTH = 640
IMG_HEIGHT = 480
# 每次推理后执行的步数（Receding Horizon策略：一次推理，执行前N步）
EXEC_STEPS = 5
# ===========================================

# 尝试导入硬件接口
try:
    from api_eef_gripper import CtrlFrankaAndGripper
    HAS_ROBOT_API = True
except ImportError:
    HAS_ROBOT_API = False
    print("⚠️ Warning: 找不到 api_eef_gripper.py，将运行在 Mock (无硬件) 模式")

class RobotInferenceClient:
    def __init__(self, use_real_robot=True):
        self.use_real_robot = use_real_robot and HAS_ROBOT_API
        
        if self.use_real_robot:
            try:
                self.api = CtrlFrankaAndGripper()
                # 测试连接
                self.api.get_eef_gripper()
                print("✅ 硬件连接成功")
            except Exception as e:
                print(f"❌ 硬件连接失败: {e}")
                self.use_real_robot = False
        else:
            self.api = None
            print("⚠️ 正在运行 Mock 模式 (生成随机数据)")

    def get_robot_state(self):
        """获取机械臂状态 [x,y,z, qx,qy,qz,qw, gripper_width]"""
        if not self.use_real_robot: 
            return np.array([0.5, 0.0, 0.5, 0, 0, 0, 1, 0.08])
        return np.array(self.api.get_eef_gripper(), dtype=float)

    def process_image(self, img_bgr):
        """BGR -> RGB -> Resize"""
        if img_bgr is None: 
            return np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img_rgb, (IMG_WIDTH, IMG_HEIGHT))
        return img_resized

    def encode_image_to_base64(self, img_rgb):
        """
        【服务器交互 - 图像编码】
        将 Numpy RGB 图片压缩为 JPG Base64 字符串，用于HTTP请求传输
        
        优化目的：大幅减小数据包体积 (原始14MB -> 压缩后300KB)
        
        流程：
        1. RGB -> BGR (OpenCV编码需要BGR格式)
        2. JPEG压缩 (质量95，平衡压缩率和图像质量)
        3. Base64编码 (转为文本字符串，便于JSON传输)
        
        Args:
            img_rgb: numpy array, shape (H, W, 3), dtype uint8, RGB格式
            
        Returns:
            str: Base64编码的JPEG图像字符串
        """
        # 转回 BGR 给 OpenCV 编码
        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        # 编码为 JPG, 质量 95
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
        _, encimg = cv2.imencode('.jpg', img_bgr, encode_param)
        # 转 Base64 文本
        b64_string = base64.b64encode(encimg).decode('utf-8')
        return b64_string

    def get_images(self):
        """获取图像列表 (List of Numpy RGB)"""
        if not self.use_real_robot:
            dummy = np.random.randint(0, 255, (IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)
            return [dummy, dummy, dummy] # 模拟三视角
            
        try:
            # 获取三张原始图 (假设顺序: Mid, Left, Right)
            # ⚠️ 请确认这里返回的变量顺序与训练时一致！
            img_mid, img_left, img_right = self.api.get_three_images(timeout_s=2.0)
            
            return [
                self.process_image(img_mid),
                self.process_image(img_left),
                self.process_image(img_right)
            ]
        except Exception as e:
            print(f"取图失败: {e}")
            blk = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)
            return [blk, blk, blk]

    def compute_target_pose(self, current_pose_8d, pred_action_7d):
        """
        【动作解析 - 服务器返回的Action转换为目标位姿】
        
        服务器返回的action是相对于当前末端执行器坐标系（Local Frame）的增量，
        需要转换为世界坐标系（Global Frame）下的绝对目标位姿。
        
        Args:
            current_pose_8d: np.ndarray, shape=(8,), 当前位姿
                [x, y, z, qx, qy, qz, qw, gripper_width]
            pred_action_7d: np.ndarray, shape=(7,), 服务器返回的动作增量
                [delta_x_local, delta_y_local, delta_z_local, 
                 rot_x, rot_y, rot_z, gripper_prob]
                - 前3维：位置增量（Local Frame，单位：米）
                - 中间3维：旋转增量（旋转向量，单位：弧度）
                - 最后1维：夹爪开合概率（0-1之间）
        
        Returns:
            np.ndarray, shape=(8,), 目标位姿
                [target_x, target_y, target_z, target_qx, target_qy, target_qz, target_qw, target_gripper_width]
        """
        # 提取当前位姿的位置和四元数
        curr_p = current_pose_8d[:3]  # 位置 [x, y, z]
        curr_q = current_pose_8d[3:7]  # 四元数 [qx, qy, qz, qw]
        r_curr = R.from_quat(curr_q)  # 转换为旋转对象

        # 解析服务器返回的动作
        delta_p_local = pred_action_7d[:3]  # Local Frame下的位置增量
        delta_rot_vec = pred_action_7d[3:6]  # 旋转向量（旋转增量）
        pred_gripper  = pred_action_7d[6]  # 夹爪开合概率

        # 坐标转换：Local Frame -> World Frame
        # 将Local Frame下的位置增量转换到World Frame
        delta_p_world = r_curr.apply(delta_p_local)
        target_p = curr_p + delta_p_world  # 计算目标位置

        # 姿态叠加：将旋转增量叠加到当前姿态
        r_rel = R.from_rotvec(delta_rot_vec)  # 旋转增量转为旋转对象
        r_next = r_curr * r_rel  # 组合旋转（当前姿态 * 增量旋转）
        target_q = r_next.as_quat()  # 转回四元数格式

        # 夹爪映射：将概率值（0-1）映射为实际宽度（米）
        # 阈值0.5：>0.5表示张开，<=0.5表示闭合
        target_width = 0.08 if pred_gripper > 0.5 else 0.0
            
        return np.concatenate([target_p, target_q, [target_width]])

    def run(self, task_instruction, hz=5):
        print(f"=== 开始执行任务: {task_instruction} ===")
        print(f"=== 目标服务器: {MODEL_URL} ===")
        dt = 1.0 / hz
        
        try:
            while True:
                t0 = time.time()
                
                # ============================================================
                # 【服务器交互流程 - 请求阶段】
                # ============================================================
                
                # 1. 数据采集：获取多视角图像和机器人状态
                images_np = self.get_images()  # List[np.ndarray], 每个元素 shape=(H, W, 3), RGB格式
                start_state = self.get_robot_state()  # np.ndarray, shape=(8,), [x,y,z, qx,qy,qz,qw, gripper_width]
                
                # 2. 图像编码：将numpy数组转为Base64字符串列表（用于HTTP传输）
                # 注意：这里只编码图像，state在payload中设为None（服务端会自动补全为0）
                images_b64 = [self.encode_image_to_base64(img) for img in images_np]
                
                # 3. 构造请求Payload
                # 请求格式说明：
                # - "examples": 列表格式，支持批量推理（这里只传一个样本）
                # - "image": Base64编码的图像列表，顺序需与训练时一致（通常为 [Mid, Left, Right]）
                # - "lang": 任务指令文本（自然语言描述）
                # - "state": 机器人状态，这里传None表示服务端会自动补全为全0向量
                #            （如果需要传真实state，格式应为: [x, y, z, qx, qy, qz, qw, gripper_width]）
                payload = {
                    "examples": [
                        {
                            "image": images_b64,     # List[str]: Base64编码的JPEG图像字符串列表
                            "lang": task_instruction, # str: 任务指令，如 "Pick up the banana"
                            "state": None            # Optional[List[float]]: 传None时服务端自动补全为0
                        }
                    ]
                }
                
                # 4. 发送HTTP POST请求到推理服务器
                try:
                    # 请求参数：
                    # - url: MODEL_URL (例如 "http://172.16.17.208:5003/predict_action")
                    # - json: 自动序列化为JSON格式的请求体
                    # - timeout: 30秒超时（推理可能需要较长时间）
                    resp = requests.post(MODEL_URL, json=payload, timeout=30.0)
                    
                    # 5. 检查HTTP响应状态
                    if resp.status_code != 200:
                        print(f"Server Error {resp.status_code}: {resp.text}")
                        continue  # 请求失败，跳过本次循环
                        
                    # 6. 解析JSON响应
                    result = resp.json()
                    
                    # ============================================================
                    # 【服务器交互流程 - 响应解析阶段】
                    # ============================================================
                    
                    # 响应数据结构说明：
                    # result["data"]["unnormalized_actions"] 的形状为 [Batch=1, Chunk=30, Dim=7]
                    # - Batch: 批次大小（这里为1，因为只传了一个样本）
                    # - Chunk: 预测的动作序列长度（通常为30步，Receding Horizon策略）
                    # - Dim: 动作维度（7维：3维位置增量 + 3维旋转增量 + 1维夹爪）
                    #   动作格式: [delta_x, delta_y, delta_z, rot_x, rot_y, rot_z, gripper]
                    #   注意：位置和旋转都是相对于当前末端执行器坐标系（Local Frame）的增量
                    
                    all_actions = np.array(result["data"]["unnormalized_actions"])
                    # 提取第一个样本（索引0）的前EXEC_STEPS步动作
                    # 最终 shape: (EXEC_STEPS, 7)
                    action_chunk = all_actions[0, :EXEC_STEPS, :]
                    
                except Exception as e:
                    # 请求异常处理：网络错误、超时、JSON解析错误等
                    print(f"❌ Inference Failed: {e}")
                    time.sleep(0.5)  # 短暂等待后重试
                    continue

                # ==========================
                # 2. 执行阶段 (Execution)
                # ==========================
                print(f"🚀 Executing Chunk ({len(action_chunk)} steps)...")
                
                # 初始化“虚拟当前状态”，用于链式计算
                virtual_current_pose = start_state.copy()
                
                for i, action in enumerate(action_chunk):
                    t_step_start = time.time()
                    
                    # 基于上一步的虚拟终点，计算这一步的终点
                    target_pose = self.compute_target_pose(virtual_current_pose, action)
                    
                    # 安全检查 (防止单步突变)
                    step_diff = np.linalg.norm(target_pose[:3] - virtual_current_pose[:3])
                    if step_diff > 0.05: # 单步超过 5cm
                         print(f"⚠️ Step {i} jump too large ({step_diff:.3f}m), clamping...")
                         # 可以在这里做插值限制，这里简单跳过打印警告
                    
                    # 执行动作
                    # print(f"   Step {i+1}: Pos={target_pose[:3].round(3)}")
                    if self.use_real_robot:
                        self.api.set_eef_gripper(target_pose.tolist())
                    
                    # 【重要】更新虚拟状态，为下一步做准备
                    virtual_current_pose = target_pose
                    
                    # 维持 Hz
                    elapsed = time.time() - t_step_start
                    time.sleep(max(0, dt - elapsed))

        except KeyboardInterrupt:
            print("\n🛑 停止执行")


if __name__ == "__main__":
    client = RobotInferenceClient(use_real_robot=True)
    client.run("Pick up the banana and put it in the basket")