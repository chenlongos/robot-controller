# src/base/mecanum_base.py
"""麦轮底盘实现模块"""

import time
import threading
import logging
from typing import Dict

from src.abstract.base_interface import BaseInterface
from .motor import Motor

logger = logging.getLogger(__name__)


class MecanumBase(BaseInterface):
    """麦轮底盘实现 - 支持全向移动"""
    
    def __init__(self, config: Dict):
        """
        :param config: 配置字典
        """
        motors_config = config.get('motors', {})
        self.motors = {}
        for name, motor_config in motors_config.items():
            # 正确调用 Motor 类的构造函数
            self.motors[name] = Motor(
                name=motor_config['name'],
                gpio_in2=motor_config['gpio_in2'],
                gpio_in1=motor_config['gpio_in1'],
                gpio_phase_a=motor_config['gpio_phase_a'],
                gpio_phase_b=motor_config['gpio_phase_b'],
                pwm_chip=motor_config['pwm_chip'],
                pwm_channel=motor_config['pwm_channel'],
                kp=motor_config['kp'],
                ki=motor_config['ki'],
                kd=motor_config['kd'],
                direction_inverted=motor_config.get('direction_inverted', False)
            )
        
        self.max_speed = config.get('max_speed', 400)
        self.last_time = time.perf_counter()
        self.vx = 0  # 前后速度
        self.vy = 0  # 左右速度
        self.vw = 0  # 旋转速度
        self.LOOP_TIME = 0.001
        self.running = True
        self.thread = threading.Thread(target=self._control_loop)
        self.thread.daemon = True
        self.thread.start()
        logger.info("Mecanum Base initialized")
    
    def _control_loop(self):
        """麦轮运动学控制循环"""
        while self.running:
            now = time.perf_counter()
            dt = max(0.001, now - self.last_time)
            self.last_time = now

            # 麦轮运动学解算 (标准布局：左前/右后辊子朝内)
            v_fl = self.vx - self.vy - self.vw
            v_fr = self.vx + self.vy + self.vw
            v_bl = self.vx + self.vy - self.vw 
            v_br = self.vx - self.vy + self.vw 
            
            # 归一化
            max_val = max(abs(v_fl), abs(v_fr), abs(v_bl), abs(v_br))
            
            if max_val > self.max_speed:
                scale = self.max_speed / max_val
                v_fl *= scale
                v_fr *= scale
                v_bl *= scale
                v_br *= scale
                
            # 下发目标速度给每个电机的 PID
            for name, motor in self.motors.items():
                if 'left' in name.lower():
                    motor.set_speed(v_bl)
                elif 'right' in name.lower():
                    motor.set_speed(v_fr)
                motor.update(dt)
            time.sleep(self.LOOP_TIME)
    
    def move(self, x: float, y: float, w: float) -> None:
        """控制底盘运动
        
        Args:
            x: 前后方向速度 (m/s)
            y: 左右方向速度 (m/s)
            w: 旋转角速度 (rad/s)
        """
        self.vx = x
        self.vy = y
        self.vw = w
    
    def stop(self) -> None:
        """停止运动"""
        self.vx = 0
        self.vy = 0
        self.vw = 0
    
    def cleanup(self) -> None:
        """释放资源"""
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1)
        for motor in self.motors.values():
            motor.cleanup()
        logger.info("Mecanum Base cleaned up")
