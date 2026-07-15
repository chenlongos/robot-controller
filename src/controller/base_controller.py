# src/controller/base_controller.py
"""底盘控制器模块"""

from typing import Dict, Any
from src.abstract.base_interface import BaseInterface
from src.config_loader import ControlConfig


class BaseController:
    """底盘控制器"""
    
    def __init__(self, base: BaseInterface, config: Any) -> None:
        self.base: BaseInterface = base
        self.config: Any = config
        
        if isinstance(config, ControlConfig):
            self._config_dict = {
                'search_rotation_speed': config.search_rotation_speed,
                'kp_angle': config.kp_angle,
                'kp_dist': config.kp_dist,
                'max_speed': config.max_speed,
                'wheel_base': config.wheel_base,
                'max_linear_speed': config.max_linear_speed,
                'target_x': config.target_x,
                'target_distance': config.target_distance,
                'approach_kp': config.approach_kp,
                'approach_ki': config.approach_ki,
                'approach_kd': config.approach_kd,
                'approach_kp_angle': config.approach_kp_angle,
            }
        elif isinstance(config, dict):
            self._config_dict = config.copy()
        else:
            self._config_dict = {}
        
        self.target_x: float = 0.0
        self.target_y: float = 0.0
        self.target_w: float = 0.0
        
        self._prev_distance = None
        self._integral = 0.0
    
    def search(self) -> Dict[str, float]:
        """搜索模式 - 360度旋转扫描，返回角速度指令"""
        search_speed = self._config_dict.get('search_rotation_speed', 0.3)
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
        
        kp_angle = self._config_dict.get('kp_angle', 0.5)
        max_linear_speed = self._config_dict.get('max_linear_speed', 0.4)
        target_x = self._config_dict.get('target_x', 0.0)
        
        angular_speed = kp_angle * (error_x - target_x)
        angular_speed = max(-1.0, min(1.0, angular_speed))
        
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
            线速度和角速度指令
        """
        distance = observation.get("target_distance", 1.0)
        error_x = observation.get("target_offset_x", 0.0)
        
        max_speed = self._config_dict.get('max_speed', 0.3)
        target_x = self._config_dict.get('target_x', 0.0)
        target_distance = self._config_dict.get('target_distance', 0.2)
        
        kp_dist = self._config_dict.get('approach_kp', 2.0)
        ki_dist = self._config_dict.get('approach_ki', 0.5)
        kd_dist = self._config_dict.get('approach_kd', 0.1)
        kp_angle = self._config_dict.get('approach_kp_angle', 0.5)
        
        angular_speed = kp_angle * (error_x - target_x)
        angular_speed = max(-0.8, min(0.8, angular_speed))
        
        error_dist = distance - target_distance
        
        max_integral = max_speed / ki_dist if ki_dist > 0 else float('inf')
        self._integral += error_dist
        self._integral = max(-max_integral, min(max_integral, self._integral))
        
        if self._prev_distance is not None:
            derivative = distance - self._prev_distance
        else:
            derivative = 0.0
        self._prev_distance = distance
        
        speed = kp_dist * error_dist + ki_dist * self._integral + kd_dist * derivative
        
        min_speed = 0.09
        if speed > 0:
            linear_speed = max(min_speed, min(max_speed, speed))
        else:
            linear_speed = min(-min_speed, max(-max_speed, speed))
        
        self.base.move(linear_speed, 0.0, angular_speed)
        return {"x": linear_speed, "w": angular_speed}
    
    def reset_pid(self) -> None:
        """重置PID状态"""
        self._prev_distance = None
        self._integral = 0.0
    
    def stop(self) -> None:
        """停止底盘"""
        self.base.stop()
        self.reset_pid()
    
    def move(self, x: float, y: float, w: float) -> None:
        """直接控制底盘运动"""
        self.base.move(x, y, w)