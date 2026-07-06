# src/base/differential_base.py
"""差分底盘实现模块"""

import time
import threading
import logging
from typing import Dict

from src.abstract.base_interface import BaseInterface
from .drivers import MotorDriverProtocol

logger = logging.getLogger(__name__)


class DifferentialBase(BaseInterface):
    """差分底盘实现 - 传统两轮差速驱动"""
    
    def __init__(self, config: Dict):
        """
        :param config: 配置字典
        """
        self.driver: MotorDriverProtocol = config.get('driver')
        self.wheel_radius = config.get('wheel_radius', 0.042)
        self.wheel_base = config.get('wheel_base', 0.195)
        self.max_linear_speed = config.get('max_linear_speed', 0.4)
        self.max_angular_speed = config.get('max_angular_speed', 1.0)
        
        self.last_time = time.perf_counter()
        self.vx = 0.0  # 前后速度 (m/s)
        self.vy = 0.0  # 左右速度（差分底盘为0）
        self.vw = 0.0  # 旋转速度 (rad/s)
        self.LOOP_TIME = 0.001
        self.running = True
        self.thread = threading.Thread(target=self._control_loop)
        self.thread.daemon = True
        self.thread.start()
        logger.info(f"Differential Base initialized (wheel_radius={self.wheel_radius}m, wheel_base={self.wheel_base}m)")
    
    def _control_loop(self):
        """差分运动学控制循环"""
        while self.running:
            now = time.perf_counter()
            dt = max(0.001, now - self.last_time)
            self.last_time = now

            vx_rpm = (self.vx * 60) / (2 * 3.14159 * self.wheel_radius)
            vw_rad = self.vw
            wheel_speed_diff = vw_rad * self.wheel_base / 2
            vw_rpm = wheel_speed_diff * 60 / (2 * 3.14159 * self.wheel_radius)
            
            v_left = vx_rpm - vw_rpm
            v_right = vx_rpm + vw_rpm
            
            max_linear_rpm = (self.max_linear_speed * 60) / (2 * 3.14159 * self.wheel_radius)
            max_angular_rpm = (self.max_angular_speed * self.wheel_base / 2 * 60) / (2 * 3.14159 * self.wheel_radius)
            
            if abs(self.vx) > 0.001 and abs(self.vw) > 0.001:
                max_rpm = max(max_linear_rpm, max_angular_rpm)
            elif abs(self.vx) > 0.001:
                max_rpm = max_linear_rpm
            else:
                max_rpm = max_angular_rpm
            
            max_val = max(abs(v_left), abs(v_right))
            
            if max_val > max_rpm:
                scale = max_rpm / max_val
                v_left *= scale
                v_right *= scale
            
            left_pwm = int((v_left / max_rpm) * 100)
            right_pwm = int((v_right / max_rpm) * 100)
            
            self.driver.set_speeds(left_pwm, right_pwm)
            self.driver.update(dt)
            
            time.sleep(self.LOOP_TIME)
    
    def move(self, x: float, y: float, w: float) -> None:
        """控制底盘运动
        
        Args:
            x: 前后方向速度 (m/s)，正值前进，负值后退
            y: 左右方向速度 (m/s) - 差分底盘忽略此参数
            w: 旋转角速度 (rad/s)，正值顺时针，负值逆时针
        """
        self.vx = max(-self.max_linear_speed, min(self.max_linear_speed, x))
        self.vw = max(-self.max_angular_speed, min(self.max_angular_speed, w))
    
    def stop(self) -> None:
        """停止运动"""
        self.vx = 0.0
        self.vy = 0.0
        self.vw = 0.0
        self.driver.stop()
    
    def cleanup(self) -> None:
        """释放资源"""
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1)
        self.driver.cleanup()
        logger.info("Differential Base cleaned up")