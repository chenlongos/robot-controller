# src/abstract/camera_interface.py
"""摄像头抽象基类模块"""

from abc import ABC, abstractmethod
import numpy as np
from typing import Dict


class CameraInterface(ABC):
    """摄像头抽象基类"""
    
    @abstractmethod
    def capture(self) -> np.ndarray:
        """捕获一帧图像"""
        pass
    
    @abstractmethod
    def get_intrinsics(self) -> Dict[str, float]:
        """获取相机内参"""
        pass
    
    @abstractmethod
    def release(self) -> None:
        """释放资源"""
        pass
    
    @abstractmethod
    def is_opened(self) -> bool:
        """检查摄像头是否打开"""
        pass
