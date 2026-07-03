# src/main.py

import os
import logging
import time
import cv2
from dataclasses import dataclass, field
from src.config_loader import load_config
from src.camera.usb_camera import USBCamera
from src.controller.vision_module import VisionModule
from src.controller.base_controller import BaseController
from src.abstract.base_factory import BaseFactory
from src.abstract.camera_factory import CameraFactory
from src.state_machine import StateMachine, RobotStatus

# 加载配置
config = load_config()

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# 根据配置设置日志
logging.basicConfig(
    level=getattr(logging, config.system.log_level),
    format='%(asctime)s - %(levelname)s - %(message)s',
    filename='th.log', filemode='w'
)

# 注册工厂类
CameraFactory.register_camera("usb", USBCamera)


@dataclass
class Robot:
    """机器人类 - 通过配置文件创建，无需硬编码"""
    status: str = "chase_tennis"  # 机器人状态
    box_cur_width: int = 0
    box_cur_height: int = 0
    box_cur_x: int = 0
    frame_height: int = 0

    controller: BaseController = field(init=False)  # 使用控制器类型
    camera = None

    def __post_init__(self):
        """根据配置初始化机器人底盘"""
        try:
            base_config = config.device.hardware.base
            base_type = base_config.type
            base = BaseFactory.create_base(base_type, base_config)
            self.controller = BaseController(base, config.control)
            logging.info("底盘控制器初始化成功")
        except Exception as e:
            logging.error(f"底盘控制器初始化失败: {e}")
            raise
    
    def set_motor_speed(self, result):
        """根据视觉检测结果设置电机速度"""
        IMG_WIDTH = config.device.parameters.frame_width
        MAX_SPEED = config.control.max_speed
        MIN_SPEED = MAX_SPEED // config.control.min_speed_ratio
        WHEEL_BASE = config.control.wheel_base
        TARGET_X = IMG_WIDTH // 2
        TARGET_W = int(config.vision.tennis_width_far * 0.6 + config.vision.tennis_width_near * 0.4)

        Kp_dist = config.control.kp_dist
        Kp_angle = config.control.kp_angle

        result_sorted = sorted(result, key=lambda x: x['w'], reverse=True)
        box = result_sorted[0]
        x, w, h = box["x"], box["w"], box["h"]
        self.box_cur_height = h
        self.box_cur_x = x
        self.box_cur_width = w
        logging.info("(box_cur_x, box_cur_width, box_cur_height) ==> %d, %d, %d", 
                     self.box_cur_x, self.box_cur_width, self.box_cur_height)

        if config.vision.tennis_width_far < self.box_cur_width < config.vision.tennis_width_near:
            self.controller.stop()
            return 0, 0

        # 1. 计算偏差
        error_x = (x + w / 2) - TARGET_X
        error_w = w - TARGET_W

        # 2. 计算线性速度和角速度
        raw_v = -Kp_dist * error_w
        raw_omega = -Kp_angle * error_x

        # 3. 动态限速
        turn_factor = abs(error_x) / (IMG_WIDTH / 2)
        if turn_factor > 0.8:
            max_v = MIN_SPEED * 0.3
        else:
            max_v = MAX_SPEED

        v = max(min(raw_v, max_v), -max_v)

        if abs(v) < MIN_SPEED and abs(v) > 0:
            v = MIN_SPEED if v > 0 else -MIN_SPEED

        diff_speed = raw_omega * WHEEL_BASE

        # 使用 move 方法设置速度（x, y, w）
        self.controller.move(v, 0.0, diff_speed)

        return int(v), int(diff_speed)
    
    def idle(self):
        """空闲状态：旋转搜索"""
        self.controller.search()


def generate_action(state: RobotStatus, observation: dict) -> dict:
    """根据当前状态生成动作"""
    action = {}
    
    if state == RobotStatus.SEARCH_TENNIS:
        action = {"type": "search", "target": "tennis"}
    elif state == RobotStatus.APPROACH_TENNIS:
        action = {"type": "approach", "target": "tennis"}
    elif state == RobotStatus.TRACK_TENNIS:
        action = {"type": "track", "target": "tennis"}
    elif state == RobotStatus.PICK:
        action = {"type": "pick"}
    elif state == RobotStatus.SEARCH_BUCKET:
        action = {"type": "search", "target": "bucket"}
    elif state == RobotStatus.APPROACH_BUCKET:
        action = {"type": "approach", "target": "bucket"}
    elif state == RobotStatus.TRACK_BUCKET:
        action = {"type": "track", "target": "bucket"}
    elif state == RobotStatus.PUT_BALL:
        action = {"type": "put"}
    
    return action


def main():
    """主控制循环"""
    all_timings = []    # 每帧的处理时间
    
    # 初始化摄像头
    logging.info("正在初始化摄像头...")
    camera_config = config.device.hardware.camera
    camera_type = camera_config.type
    camera = CameraFactory.create_camera(camera_type, camera_config)
    
    if not camera.is_opened():
        raise IOError("无法打开摄像头")
    
    # 初始化视觉模块
    logging.info("正在初始化视觉模块...")
    vision_module = VisionModule(config)
    
    # 初始化机器人（完全通过配置文件创建）
    logging.info("正在初始化机器人...")
    robot = Robot()
    robot.camera = camera

    # 初始化状态机
    logging.info("正在初始化状态机...")
    state_machine = StateMachine()

    try:
        while True:
            start_time = time.time() * 1000
            
            # 捕获帧
            frame = camera.capture()
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

            # 执行视觉推理
            logging.debug(f"解算开始")
            result = vision_module.infer(frame)
            logging.debug(f"解算完成")

            # 构建观测数据
            observation = {
                "camera": frame,
                "tennis_detected": len(result) > 0,
                "initialized": True,
                "timestamp": time.time()
            }
            
            if result:
                box = sorted(result, key=lambda x: x['w'], reverse=True)[0]
                observation["tennis_distance"] = 1.0 - (box["w"] / target_width)
                observation["target_offset_x"] = (box["x"] + box["w"] / 2) - target_width / 2

            # 状态机更新
            current_state = state_machine.get_state()
            next_state = state_machine.transition(observation)
            
            if next_state != current_state:
                logging.info(f"状态转换: {current_state} -> {next_state}")
                state_machine.set_state(next_state)

            # 根据状态执行动作
            action = generate_action(state_machine.get_state(), observation)
            logging.info(f"当前状态: {state_machine.get_state()}, 动作: {action}")

            # 执行动作
            if state_machine.get_state() in [RobotStatus.SEARCH_TENNIS, RobotStatus.SEARCH_BUCKET]:
                robot.idle()
            elif result:
                v, diff_speed = robot.set_motor_speed(result)
                logging.info(f"set_motor_speed: (v, diff_speed) ==> {v}, {diff_speed}")
            else:
                robot.idle()
            
            # 统计处理时间
            all_timings.append(int(time.time() * 1000 - start_time))
            avg_time = int(sum(all_timings) / len(all_timings) if all_timings else 0)
            min_time = min(all_timings) if all_timings else 0
            max_time = max(all_timings) if all_timings else 0
            logging.info(f"当前帧处理时间: {all_timings[-1]} ms, 平均: {avg_time} ms, 最小: {min_time} ms, 最大: {max_time} ms")

    except KeyboardInterrupt:
        logging.info("收到中断信号，正在退出...")
    
    finally:
        # 释放资源
        vision_module.release()
        camera.release()
        robot.controller.stop()
        robot.controller.base.cleanup()
        logging.info("资源已释放")


if __name__ == "__main__":
    main()
