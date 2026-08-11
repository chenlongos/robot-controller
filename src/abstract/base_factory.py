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
    def _create_driver(config):
        """根据配置创建驱动实例"""
        driver_type = config.driver
        
        if driver_type == "d24a_jgb37":
            from src.base.drivers import D24aJgb37Driver
            return D24aJgb37Driver(config.motors, config.pid)
        elif driver_type == "esp32_c3_tt":
            from src.base.drivers import Esp32C3TtDriver
            return Esp32C3TtDriver(config.uart)
        else:
            raise ValueError(f"不支持的驱动类型: {driver_type}")
    
    @staticmethod
    def create_base(base_type: str, config) -> BaseInterface:
        """根据配置创建底盘实例"""
        base_type = base_type.lower()
        
        if base_type not in BaseFactory._base_registry:
            raise ValueError(f"不支持的底盘类型: {base_type}")
        
        driver = BaseFactory._create_driver(config)
        
        base_config_dict = {
            'driver': driver,
            'wheel_radius': config.wheel_radius,
            'wheel_base': config.wheel_base,
            'max_linear_speed': config.max_linear_speed,
            'max_angular_speed': config.max_angular_speed,
            'pid': config.pid,
            'direction_forward': config.uart.direction_forward if config.uart else 1,
            'linear_k': config.linear_k,
            'rotation_k': config.rotation_k,
        }
        
        return BaseFactory._base_registry[base_type](base_config_dict)