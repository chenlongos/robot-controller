# src/abstract/arm_interface.py
"""机械臂抽象基类模块

机械臂的硬件层控制仅提供最底层的关节级控制：
- 关节角度控制与读取
- 夹爪位置控制与读取
- 运动状态查询与停止
- 连接生命周期管理

笛卡尔坐标移动、抓取动作等高级控制由上层控制层实现。
"""

from abc import ABC, abstractmethod
from typing import List, Tuple


class ArmInterface(ABC):
    """机械臂抽象基类"""
    
    JOINT_NAMES: Tuple[str, ...] = ()
    
    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """检查机械臂是否已连接"""
        pass
    
    @abstractmethod
    def connect(self, calibrate: bool = True) -> None:
        """连接机械臂
        
        Args:
            calibrate: 是否执行校准 (默认: True)
        """
        pass
    
    @abstractmethod
    def disconnect(self) -> None:
        """断开连接"""
        pass
    
    @abstractmethod
    def move_to_joint_positions(self, positions: List[float]) -> None:
        """移动到关节位置
        
        Args:
            positions: 关节目标位置列表，顺序与JOINT_NAMES一致
        """
        pass
    
    @abstractmethod
    def get_joint_positions(self) -> List[float]:
        """获取当前关节位置
        
        Returns:
            关节位置列表，顺序与JOINT_NAMES一致
        """
        pass
    
    @abstractmethod
    def get_gripper_position(self) -> float:
        """获取夹爪位置
        
        Returns:
            夹爪位置 (0-100)
        """
        pass
    
    @abstractmethod
    def is_moving(self) -> bool:
        """检查是否正在移动"""
        pass
    
    @abstractmethod
    def stop(self) -> None:
        """停止运动"""
        pass