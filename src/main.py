# src/main.py

import os
import logging
import time
import cv2
import yaml
from dataclasses import dataclass, field
from src.config_loader import load_config
from src.controller.vision_module import VisionModule
from src.controller.base_controller import BaseController
from src.controller.arm_controller import ArmController
from src.abstract.base_factory import BaseFactory
from src.abstract.camera_factory import CameraFactory
from src.abstract.camera_interface import CameraInterface
from src.abstract.arm_factory import ArmFactory
from src.state_machine import StateMachine, RobotStatus
from src.arm_action_config import angles_to_sequences
from src.web.webrtc_server import start_webrtc_server, push_frame, is_available as webrtc_available
import src.base
import src.camera

# 加载配置，robot_id 从 config/common.yaml 自动读取
config = load_config()

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# 根据配置设置日志
logging.basicConfig(
    level=getattr(logging, config.system.log_level),
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('th.log', mode='w'),
        logging.StreamHandler()
    ]
)

def load_calibration_params(robot_name: str, config_dir: str = 'config') -> dict:
    robot_type = robot_name.split("-")[0]
    calibration_file = os.path.join(config_dir, f'calibration_{robot_type}_distance.yaml')
    
    if os.path.exists(calibration_file):
        try:
            with open(calibration_file, 'r') as f:
                data = yaml.safe_load(f)
                return {
                    'M': data.get('M', 6042.48),
                    'C': data.get('C', -2.83)
                }
        except Exception as e:
            logging.error(f"加载校准文件失败: {e}")
    
    return {'M': 6042.48, 'C': -2.83}


def calculate_distance(bbox_width: int, M: float = None, C: float = None) -> float:
    if bbox_width <= 0:
        return float('inf')
    
    if M is None:
        M = 6042.48
    if C is None:
        C = -2.83
    
    distance_cm = M / bbox_width + C
    distance_m = distance_cm / 100.0
    
    return distance_m


@dataclass
class Robot:
    """机器人类 - 通过配置文件创建，无需硬编码"""
    status: str = "chase_tennis"  # 机器人状态
    box_cur_width: int = 0
    box_cur_height: int = 0
    box_cur_x: int = 0
    frame_height: int = 0
    target_type: str = "tennis"  # 当前跟踪目标类型: tennis, bucket

    controller: BaseController = field(init=False)  # 使用控制器类型
    arm_controller: ArmController = field(init=False)  # 机械臂控制器
    camera: CameraInterface = field(init=False)  # 摄像头

    def __post_init__(self):
        """根据配置初始化机器人底盘、机械臂和摄像头"""
        try:
            base_config = config.device.hardware.base
            base_type = base_config.type
            base = BaseFactory.create_base(base_type, base_config)
            self.controller = BaseController(base, config.control)
            logging.info("底盘控制器初始化成功")
        except Exception as e:
            logging.error(f"底盘控制器初始化失败: {e}")
            raise
        
        try:
            arm_config = config.device.hardware.arm
            arm_type = arm_config.type
            arm = ArmFactory.create_arm(arm_type, arm_config)
            arm.connect()
            self.arm_controller = ArmController(arm, robot_name=config.system.robot_id)
            logging.info("机械臂控制器初始化成功")
        except Exception as e:
            logging.error(f"机械臂控制器初始化失败: {e}")
            raise
        
        try:
            camera_config = config.device.hardware.camera
            camera_type = camera_config.type
            self.camera = CameraFactory.create_camera(camera_type, camera_config)
            logging.info("摄像头初始化成功")
        except Exception as e:
            logging.error(f"摄像头初始化失败: {e}")
            raise
    
    
    def idle(self):
        """空闲状态：旋转搜索"""
        self.controller.search()
    
    def execute_arm_action(self, action_name: str):
        """执行机械臂动作序列"""
        logging.info(f"执行机械臂动作: {action_name}")
        result = self.arm_controller.execute_action_sequence(action_name)
        logging.info(f"机械臂动作执行结果: {result}")
        return result
    
    def get_joint_positions(self):
        """获取当前所有关节位置"""
        return self.arm_controller.get_joint_positions()
    
    def get_gripper_position(self):
        """获取夹爪位置"""
        return self.arm_controller.get_gripper_position()
    
    def move_distance(self, distance: float, speed: float = 0.3) -> None:
        """精确控制前进距离
        
        Args:
            distance: 目标距离 (m)，正值前进，负值后退
            speed: 速度 (m/s)，默认0.3m/s
        """
        if distance == 0:
            return
        
        direction = 1 if distance > 0 else -1
        abs_distance = abs(distance)
        target_speed = abs(speed) * direction
        
        start_time = time.time()
        traveled_distance = 0.0
        
        self.controller.move(target_speed, 0, 0)
        
        while traveled_distance < abs_distance:
            current_time = time.time()
            elapsed_time = current_time - start_time
            traveled_distance = abs(target_speed) * elapsed_time
            time.sleep(0.01)
        
        self.controller.stop()
        logging.info(f"移动完成，目标距离: {distance:.2f}m，实际距离: {traveled_distance:.2f}m")

def main():
    """主控制循环 - 捡球循环：找球->抓球->找桶->放球->找球"""
    all_timings = []    # 每帧的处理时间
    
    robot_name = config.system.robot_id
    robot_type = robot_name.split("-")[0]
    calibration_params = load_calibration_params(robot_name)
    logging.info(f"加载校准参数: M={calibration_params['M']:.4f}, C={calibration_params['C']:.4f}")
    
    # 从 arm_angles 生成 arm_action_sequences，确保机械臂动作序列是最新的
    logging.info(f"从 arm_angles_{robot_type}.json 生成 arm_action_sequences_{robot_type}.json...")
    try:
        angles_to_sequences(robot_type)
    except FileNotFoundError:
        logging.warning(f"arm_angles_{robot_type}.json 不存在，跳过动作序列生成")
    
    # 初始化视觉模块
    logging.info("正在初始化视觉模块...")
    vision_module = VisionModule(config)
    
    # 初始化机器人（完全通过配置文件创建，包含摄像头、底盘和机械臂）
    logging.info("正在初始化机器人...")
    robot = Robot()
    
    logging.info("机械臂回到初始位置...")
    robot.execute_arm_action("origin")

    # 初始化状态机
    logging.info("正在初始化状态机...")
    sm_config = {
        'target_x': config.statemachine.target_x,
        'target_distance': config.statemachine.target_distance,
        'threshold_x': config.statemachine.threshold_x,
        'threshold_d': config.statemachine.threshold_d,
        'grip_threshold': config.statemachine.grip_threshold,
        'grip_close': robot.arm_controller.get_close_gripper_position(),
        'reach_count_threshold': config.statemachine.reach_count_threshold,
        'frame_width': config.device.parameters.frame_width,
        'bucket_edge_threshold': config.statemachine.bucket_edge_threshold
    }
    state_machine = StateMachine(sm_config)
    
    # 启动 WebRTC 服务器
    if webrtc_available():
        logging.info("启动 WebRTC 服务器...")
        start_webrtc_server(port=8080)
        logging.info("WebRTC 推流地址: http://<机器人IP>:8080")
        stream_frame_interval = 1.0 / 15.0
        last_stream_time = time.time()
    else:
        logging.warning("WebRTC 依赖未安装，跳过推流功能")

    try:
        while True:
            start_time = time.time() * 1000
            
            # 捕获帧
            frame = robot.camera.capture(flush_frames=1)
            if frame is None:
                logging.warning("无法捕获帧，重试...")
                continue
            
            height, width = frame.shape[:2]
            robot.frame_height = height
            
            # 调整帧大小
            target_width = config.device.parameters.frame_width
            if width != target_width:
                logging.warning(f"当前帧宽度 {width} 不等于 {target_width}，正在调整大小...")
                frame = cv2.resize(frame, (target_width, int(target_width * height / width)), 
                                interpolation=cv2.INTER_LINEAR)
                robot.frame_height = frame.shape[0]

            tennis_result = []
            bucket_result = []
            
            # 构建观测数据（只包含物理状态）
            observation = {
                "camera": frame,
                "timestamp": time.time(),
                "tennis_detected": False,
                "bucket_detected": False,
                "tennis_distance": float('inf'),
                "tennis_offset_x": 0.0,
                "bucket_left_edge": float('inf'),
                "bucket_right_edge": -float('inf'),
                "gripper_angle": 0
            }

            current_state = state_machine.get_state()
            
            # 网球检测
            if current_state in [RobotStatus.SEARCH_TENNIS, RobotStatus.TRACK_TENNIS]:
                tennis_result = vision_module.infer(frame)
                if tennis_result:
                    observation["tennis_detected"] = True
                    box = sorted(tennis_result, key=lambda x: x['w'], reverse=True)[0]
                    bbox_width = max(box["w"], box["h"])
                    observation["tennis_distance"] = calculate_distance(bbox_width, 
                                                                       M=calibration_params['M'],
                                                                       C=calibration_params['C'])
                    observation["tennis_offset_x"] = (box["x"] + box["w"] / 2) - target_width / 2
                    observation["tennis_left_edge"] = box["x"]
                    observation["tennis_right_edge"] = box["x"] + box["w"]
            
            # 桶检测
            elif current_state in [RobotStatus.SEARCH_BUCKET, RobotStatus.TRACK_BUCKET]:
                bucket_result = vision_module.get_bucket_local(frame, color="red")
                if bucket_result:
                    observation["bucket_detected"] = True
                    box = sorted(bucket_result, key=lambda x: x['w'], reverse=True)[0]
                    observation["bucket_left_edge"] = box["x"]
                    observation["bucket_right_edge"] = box["x"] + box["w"]

            # 状态机更新
            next_state = state_machine.transition(observation)
            
            if next_state != current_state:
                logging.info(f"状态转换: {current_state} -> {next_state}")
                state_machine.set_state(next_state)
                state_machine.lost_count = 0
                
                # 状态转换时的特殊处理
                if next_state == RobotStatus.PICK:
                    robot.controller.stop()

                    logging.info("开始抓取网球...")
                    result = robot.execute_arm_action("pick")
                    observation["gripper_angle"] = robot.get_gripper_position()
                    logging.info(f"抓取完成, 夹爪角度: {observation['gripper_angle']:.2f}")
                    
                    next_state = state_machine.transition(observation)
                    state_machine.set_state(next_state)
                    if next_state == RobotStatus.SEARCH_BUCKET:
                        logging.info(f"抓取完成，状态转换: {RobotStatus.PICK} -> {next_state}")
                        robot.target_type = "bucket"
                    else:
                        logging.info(f"抓取失败，状态转换: {RobotStatus.PICK} -> {next_state}")
                
                elif next_state == RobotStatus.PUT_BALL:
                    robot.controller.stop()
                    logging.info("开始放置网球...")
                    result = robot.execute_arm_action("put")
                    logging.info(f"放置完成")
                    
                    next_state = state_machine.transition(observation)
                    state_machine.set_state(next_state)
                    logging.info(f"放置完成，状态转换: {RobotStatus.PUT_BALL} -> {next_state}")
                    
                    if next_state in [RobotStatus.SEARCH_TENNIS, RobotStatus.TRACK_TENNIS]:
                        robot.target_type = "tennis"
                
                elif next_state in [RobotStatus.SEARCH_TENNIS, RobotStatus.TRACK_TENNIS]:
                    robot.target_type = "tennis"
                elif next_state in [RobotStatus.SEARCH_BUCKET, RobotStatus.TRACK_BUCKET]:
                    robot.target_type = "bucket"

            # 执行动作
            current_state = state_machine.get_state()
            logging.debug(f"当前状态: {current_state.value}, 网球检测: {observation['tennis_detected']}, 桶检测: {observation['bucket_detected']}")
            if current_state in [RobotStatus.SEARCH_TENNIS, RobotStatus.SEARCH_BUCKET]:
                logging.debug(f"执行搜索旋转: {current_state.value}")
                robot.idle()
            elif current_state == RobotStatus.TRACK_TENNIS:
                if observation["tennis_detected"]:
                    track_observation = {
                        "target_offset_x": observation["tennis_offset_x"],
                        "target_distance": observation["tennis_distance"]
                    }
                    command = robot.controller.track(track_observation)
                    logging.debug(f"追踪中: speed_x={command.get('x', 0):.3f}, speed_w={command.get('w', 0):.3f}")
                else:
                    logging.debug("追踪网球但未检测到，执行搜索旋转")
                    robot.idle()
            elif current_state == RobotStatus.TRACK_BUCKET:
                if observation["bucket_detected"]:
                    bucket_left_edge = observation.get("bucket_left_edge", 0)
                    bucket_right_edge = observation.get("bucket_right_edge", 0)
                    
                    edge_ok = bucket_left_edge != 0 and bucket_right_edge != target_width
                    
                    if True:
                        bucket_center_x = (bucket_left_edge + bucket_right_edge) / 2
                        bucket_offset_x = bucket_center_x - config.device.parameters.frame_width / 2
                        track_observation = {
                            "target_offset_x": bucket_offset_x,
                            "target_distance": observation.get("bucket_distance", 1.0)
                        }
                        command = robot.controller.track(track_observation)
                        logging.debug(f"追踪桶: speed_x={command.get('x', 0):.3f}, speed_w={command.get('w', 0):.3f}")
                    else:
                        logging.debug(f"桶框边缘越界，正在旋转调整: left={bucket_left_edge}, right={bucket_right_edge}")
                        search_speed = config.control.search_rotation_speed
                        if bucket_left_edge == 0:
                            robot.controller.move(0.0, 0.0,-search_speed)
                        else:
                            robot.controller.move(0.0, 0.0, +search_speed)
                else:
                    logging.debug("追踪桶但未检测到，执行搜索旋转")
                    robot.idle()
            elif current_state in [RobotStatus.PICK, RobotStatus.PUT_BALL]:
                logging.debug(f"停止移动: {current_state.value}")
                robot.controller.stop()
            
            # 统计处理时间
            all_timings.append(int(time.time() * 1000 - start_time))
            avg_time = int(sum(all_timings) / len(all_timings) if all_timings else 0)
            min_time = min(all_timings) if all_timings else 0
            max_time = max(all_timings) if all_timings else 0
            logging.debug(f"当前帧处理时间: {all_timings[-1]} ms, 平均: {avg_time} ms, 最小: {min_time} ms, 最大: {max_time} ms")
            
            # WebRTC 视频推流
            if webrtc_available():
                current_time = time.time()
                if current_time - last_stream_time >= stream_frame_interval:
                    display_frame = vision_module.draw_tracking_info(frame, tennis_result, bucket_result, 
                                                                    state_machine.get_state(), observation)
                    push_frame(display_frame)
                    last_stream_time = current_time

    except KeyboardInterrupt:
        logging.info("收到中断信号，正在退出...")
    
    finally:
        # 释放资源
        vision_module.release()
        robot.camera.release()
        robot.controller.stop()
        robot.controller.base.cleanup()
        if robot.arm_controller:
            robot.arm_controller.stop()
            robot.arm_controller.arm.disconnect()
        logging.info("资源已释放")

if __name__ == "__main__":
    main()
