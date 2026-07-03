# src/abstract/base_factory.py
"""底盘工厂模块"""

from typing import Type, Dict
from src.abstract.base_interface import BaseInterface


class BaseFactory:
    """底盘工厂类"""
    
    _base_registry: Dict[str, Type[BaseInterface]] = {}
    
    @staticmethod
    def register_base(base_type: str, base_class: Type[BaseInterface]) -> None:
        """注册底盘类型"""
        BaseFactory._base_registry[base_type.lower()] = base_class
    
    @staticmethod
    def create_base(base_type: str, config: Dict) -> BaseInterface:
        """根据配置创建底盘实例"""
        base_type = base_type.lower()
        
        if base_type not in BaseFactory._base_registry:
            raise ValueError(f"不支持的底盘类型: {base_type}")
        
        return BaseFactory._base_registry[base_type](config)
