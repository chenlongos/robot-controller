# src/controller/base_controller.py
"""底盘控制器模块"""

from typing import Dict
from src.abstract.base_interface import BaseInterface


class BaseController:
    """底盘控制器"""
    
    def __init__(self, base: BaseInterface, config: Dict) -> None:
        self.base: BaseInterface = base
        self.config: Dict = config
        self.target_x: float = 0.0
        self.target_y: float = 0.0
        self.target_w: float = 0.0
        
        self._prev_distance = None
        self._integral = 0.0
    
    def search(self) -> Dict[str, float]:
        """搜索模式 - 360度旋转扫描，返回角速度指令"""
        search_speed = self.config.get('search_rotation_speed', 0.5)
        self.base.move(0.0, 0.0, search_speed)
        return {"w": search_speed}
    
    def track(self, observation: Dict) -> Dict[str, float]:
        """跟踪目标 - 远离目标时快速接近，保持目标在视野中心
        
        Args:
            observation: 包含 target_offset_x, target_offset_y, target_distance 的观测数据
        
        Returns:
            线速度和角速度指令
        """
        error_x = observation.get("target_offset_x", 0.0)
        distance = observation.get("target_distance", float('inf'))
        
        kp_angle = self.config.get('kp_angle', 0.5)
        max_linear_speed = self.config.get('max_linear_speed', 0.4)
        approach_threshold = self.config.get('approach_threshold', 0.5)
        
        angular_speed = kp_angle * error_x
        angular_speed = max(-1.0, min(1.0, angular_speed))
        
        if distance < approach_threshold:
            linear_speed = 0.0
        else:
            offset_ratio = min(abs(error_x) / 320.0, 1.0)
            speed_factor = 1.0 - offset_ratio * 0.7
            linear_speed = max_linear_speed * speed_factor
        
        self.base.move(linear_speed, 0.0, angular_speed)
        return {"x": linear_speed, "w": angular_speed}
    
    def approach(self, observation: Dict) -> Dict[str, float]:
        """接近目标 - 近距离时精准停在指定位置（同时控制距离和角度）
        
        使用PID控制实现平滑减速和角度对准，避免冲过目标。
        
        Args:
            observation: 包含 target_distance, target_offset_x 的观测数据
        
        Returns:
            线速度和角速度指令，以及是否完成的状态
        """
        distance = observation.get("target_distance", 1.0)
        error_x = observation.get("target_offset_x", 0.0)
        
        max_speed = self.config.get('max_speed', 0.3)
        stop_threshold = self.config.get('stop_threshold', 0.05)
        angle_threshold = self.config.get('angle_threshold', 10.0)
        
        kp_dist = self.config.get('approach_kp', 2.0)
        ki_dist = self.config.get('approach_ki', 0.5)
        kd_dist = self.config.get('approach_kd', 0.1)
        kp_angle = self.config.get('approach_kp_angle', 0.5)
        
        distance_done = distance <= stop_threshold
        angle_done = abs(error_x) <= angle_threshold
        
        if distance_done and angle_done:
            self.base.move(0.0, 0.0, 0.0)
            return {"x": 0.0, "w": 0.0, "done": True}
        
        if not angle_done:
            angular_speed = kp_angle * error_x
            angular_speed = max(-0.8, min(0.8, angular_speed))
        else:
            angular_speed = 0.0
        
        if distance_done:
            linear_speed = 0.0
        else:
            self._integral += distance
            
            if self._prev_distance is not None:
                derivative = distance - self._prev_distance
            else:
                derivative = 0.0
            self._prev_distance = distance
            
            speed = kp_dist * distance + ki_dist * self._integral + kd_dist * derivative
            
            if not angle_done:
                speed *= 0.3
            
            min_speed = 0.05
            linear_speed = max(min_speed, min(max_speed, speed))
        
        self.base.move(linear_speed, 0.0, angular_speed)
        return {"x": linear_speed, "w": angular_speed, "done": False, "distance_done": distance_done, "angle_done": angle_done}
    
    def stop(self) -> None:
        """停止底盘"""
        self.base.stop()
        self._prev_distance = None
        self._integral = 0.0
    
    def move(self, x: float, y: float, w: float) -> None:
        """直接控制底盘运动"""
        self.base.move(x, y, w)