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
                'kp_dist': config.kp_dist,
                'kp_angle': config.kp_angle,
                'ki': config.ki,
                'kd': config.kd,
                'wheel_base': config.wheel_base,
                'max_linear_speed': config.max_linear_speed,
                'target_x': config.target_x,
                'target_distance': config.target_distance,
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
        """跟踪目标 - 统一的追踪/接近控制

        使用PID控制距离，远距离时快速接近，近距离时精准定位；
        角度控制根据距离自适应调整，保证远距离快速对准、近距离精确对齐。

        Args:
            observation: 包含 target_distance, target_offset_x 的观测数据

        Returns:
            线速度和角速度指令
        """
        distance = observation.get("target_distance", 1.0)
        error_x = observation.get("target_offset_x", 0.0)

        max_speed = self._config_dict.get('max_linear_speed', 0.4)
        # 角度控制的目标偏移量：可在 observation 中通过 target_offset_setpoint 覆盖
        # （例如 TRACK_TENNIS 传 0.0 以将目标对齐视野垂直中线）
        target_x = observation.get("target_offset_setpoint", self._config_dict.get('target_x', 0.0))
        target_distance = self._config_dict.get('target_distance', 0.2)

        kp_dist = self._config_dict.get('kp_dist', 2.0)
        ki_dist = self._config_dict.get('ki', 0.5)
        kd_dist = self._config_dict.get('kd', 0.1)
        kp_angle = self._config_dict.get('kp_angle', 0.5)

        error_dist = distance - target_distance

        if abs(error_dist) > 0.3:
            angular_speed = kp_angle * (target_x - error_x)
            angular_speed = max(-1.0, min(1.0, angular_speed))
        else:
            angular_speed = kp_angle * (target_x - error_x)
            angular_speed = max(-0.3, min(0.3, angular_speed))

        max_integral = max_speed / ki_dist if ki_dist > 0 else float('inf')

        if error_dist <= 0:
            self._integral = 0.0
        else:
            self._integral += error_dist
            self._integral = max(-max_integral, min(max_integral, self._integral))

        if self._prev_distance is not None:
            derivative = distance - self._prev_distance
        else:
            derivative = 0.0
        self._prev_distance = distance

        speed = kp_dist * error_dist + ki_dist * self._integral + kd_dist * derivative

        linear_speed = max(-max_speed, min(max_speed, speed))

        if abs(linear_speed) < 0.001:
            linear_speed = 0.0

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