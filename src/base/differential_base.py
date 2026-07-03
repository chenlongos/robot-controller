# src/base/differential_base.py
"""差分底盘实现模块"""

import time
import threading
import logging
from typing import Dict

from src.abstract.base_interface import BaseInterface
from .motor import Motor

logger = logging.getLogger(__name__)


class DifferentialBase(BaseInterface):
    """差分底盘实现 - 传统两轮差速驱动"""
    
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
        self.wheel_radius = config.get('wheel_radius', 0.042)  # 车轮半径（米）
        self.wheel_base = config.get('wheel_base', 0.195)      # 轮距（米）
        self.last_time = time.perf_counter()
        self.vx = 0  # 前后速度 (mm/s)
        self.vy = 0  # 左右速度（差分底盘为0）
        self.vw = 0  # 旋转速度 (度/s)
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

            # 差分运动学解算
            # vx: 底盘前后线速度 (mm/s)
            # vw: 底盘旋转角速度 (度/s)
            # 转换为电机转速 (RPM)
            
            # 将线速度 (mm/s) 转换为 RPM
            # RPM = (v * 60) / (2 * pi * r)
            # 其中 v: m/s, r: m
            vx_rpm = (self.vx / 1000) * 60 / (2 * 3.14159 * self.wheel_radius)
            
            # 将角速度 (度/s) 转换为车轮线速度差 (m/s)
            # 左右轮速度差 = vw * wheel_base / 2
            vw_rad = self.vw * 3.14159 / 180  # 度转弧度
            wheel_speed_diff = vw_rad * self.wheel_base / 2  # m/s
            
            # 转换为 RPM
            vw_rpm = wheel_speed_diff * 60 / (2 * 3.14159 * self.wheel_radius)
            
            # 计算左右轮转速
            v_left = vx_rpm - vw_rpm
            v_right = vx_rpm + vw_rpm
            
            # 归一化
            max_val = max(abs(v_left), abs(v_right))
            
            if max_val > self.max_speed:
                scale = self.max_speed / max_val
                v_left *= scale
                v_right *= scale
                
            # 下发目标速度给每个电机的 PID
            for name, motor in self.motors.items():
                if 'left' in name.lower():
                    motor.set_speed(v_left)
                elif 'right' in name.lower():
                    motor.set_speed(v_right)
                motor.update(dt)
            
            time.sleep(self.LOOP_TIME)
    
    def move(self, x: float, y: float, w: float) -> None:
        """控制底盘运动
        
        Args:
            x: 前后方向速度 (mm/s)，正值前进，负值后退
            y: 左右方向速度 (mm/s) - 差分底盘忽略此参数
            w: 旋转角速度 (度/s)，正值左转，负值右转
        """
        self.vx = x
        # 差分底盘不支持横向移动，忽略y参数
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
        logger.info("Differential Base cleaned up")
