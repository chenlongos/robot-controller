# src/abstract/arm_factory.py
"""机械臂工厂模块"""

from typing import Type, Dict
from src.abstract.arm_interface import ArmInterface


class ArmFactory:
    """机械臂工厂类"""
    
    _arm_registry: Dict[str, Type[ArmInterface]] = {}
    
    @staticmethod
    def register_arm(arm_type: str, arm_class: Type[ArmInterface]) -> None:
        """注册机械臂类型"""
        ArmFactory._arm_registry[arm_type.lower()] = arm_class
    
    @staticmethod
    def create_arm(arm_type: str, config: Dict) -> ArmInterface:
        """根据配置创建机械臂实例"""
        arm_type = arm_type.lower()
        
        if arm_type not in ArmFactory._arm_registry:
            raise ValueError(f"不支持的机械臂类型: {arm_type}")
        
        return ArmFactory._arm_registry[arm_type](config)


def _register_default_arms():
    """注册默认支持的机械臂类型"""
    try:
        from src.arm.so101_arm import SO101Arm
        ArmFactory.register_arm("so101", SO101Arm)
    except ImportError:
        pass


_register_default_arms()
