# src/abstract/arm_interface.py
"""机械臂抽象基类模块"""

from abc import ABC, abstractmethod
from typing import List


class ArmInterface(ABC):
    """机械臂抽象基类"""
    
    @abstractmethod
    def move_to_joint_positions(self, positions: List[float]) -> None:
        """移动到关节位置"""
        pass
    
    @abstractmethod
    def move_to_cartesian(self, x: float, y: float, z: float) -> None:
        """移动到笛卡尔坐标"""
        pass
    
    @abstractmethod
    def get_joint_positions(self) -> List[float]:
        """获取当前关节位置"""
        pass
    
    @abstractmethod
    def set_gripper_position(self, position: float) -> None:
        """设置夹爪位置"""
        pass
    
    @abstractmethod
    def get_gripper_position(self) -> float:
        """获取夹爪位置"""
        pass
    
    @abstractmethod
    def is_moving(self) -> bool:
        """检查是否正在移动"""
        pass
    
    @abstractmethod
    def stop(self) -> None:
        """停止运动"""
        pass
