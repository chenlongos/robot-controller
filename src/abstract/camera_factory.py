# src/abstract/camera_factory.py
"""摄像头工厂模块"""

from typing import Type, Dict
from src.abstract.camera_interface import CameraInterface


class CameraFactory:
    """摄像头工厂类"""
    
    _camera_registry: Dict[str, Type[CameraInterface]] = {}
    
    @staticmethod
    def register_camera(camera_type: str, camera_class: Type[CameraInterface]) -> None:
        """注册摄像头类型"""
        CameraFactory._camera_registry[camera_type.lower()] = camera_class
    
    @staticmethod
    def create_camera(camera_type: str, config: Dict) -> CameraInterface:
        """根据配置创建摄像头实例"""
        camera_type = camera_type.lower()
        
        if camera_type not in CameraFactory._camera_registry:
            raise ValueError(f"Unknown camera type: {camera_type}")
        
        return CameraFactory._camera_registry[camera_type](config)
