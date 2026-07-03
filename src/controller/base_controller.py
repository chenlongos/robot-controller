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
    
    def search(self) -> Dict[str, float]:
        """搜索模式 - 360度旋转扫描，返回角速度指令"""
        search_speed = self.config.get('search_rotation_speed', 0.5)
        self.base.move(0.0, 0.0, search_speed)
        return {"w": search_speed}
    
    def track(self, observation: Dict) -> Dict[str, float]:
        """跟踪目标 - 保持目标在视野中心，返回线速度和角速度"""
        # 根据观测调整速度
        error_x = observation.get("target_offset_x", 0.0)
        error_y = observation.get("target_offset_y", 0.0)
        
        kp_angle = self.config.get('kp_angle', 0.5)
        kp_dist = self.config.get('kp_dist', 0.8)
        
        angular_speed = kp_angle * error_x
        linear_speed = max(0.1, kp_dist * (1.0 - abs(error_x) / 320.0))
        
        self.base.move(linear_speed, 0.0, angular_speed)
        return {"x": linear_speed, "w": angular_speed}
    
    def approach(self, observation: Dict) -> Dict[str, float]:
        """接近目标 - 快速直线移动靠近目标，返回线速度"""
        distance = observation.get("target_distance", 1.0)
        max_speed = self.config.get('max_speed', 100)
        kp_dist = self.config.get('kp_dist', 0.8)
        
        speed = min(max_speed, kp_dist * distance)
        
        self.base.move(speed, 0.0, 0.0)
        return {"x": speed}
    
    def stop(self) -> None:
        """停止底盘"""
        self.base.stop()
    
    def move(self, x: float, y: float, w: float) -> None:
        """直接控制底盘运动"""
        self.base.move(x, y, w)
