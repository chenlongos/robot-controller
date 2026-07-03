# src/abstract/base_interface.py
"""底盘抽象基类模块"""

from abc import ABC, abstractmethod


class BaseInterface(ABC):
    """底盘抽象基类
    
    将底盘运动抽象为三个方向的叠加：
    - x: 前后方向移动速度 (m/s)，正值向前，负值向后
    - y: 左右方向移动速度 (m/s)，正值向右，负值向左
    - w: 旋转角速度 (rad/s)，正值顺时针，负值逆时针
    """
    
    @abstractmethod
    def move(self, x: float, y: float, w: float) -> None:
        """控制底盘运动
        
        Args:
            x: 前后方向速度 (m/s)
            y: 左右方向速度 (m/s)
            w: 旋转角速度 (rad/s)
        """
        pass
    
    @abstractmethod
    def stop(self) -> None:
        """停止运动"""
        pass
    
    @abstractmethod
    def cleanup(self) -> None:
        """清理资源"""
        pass
