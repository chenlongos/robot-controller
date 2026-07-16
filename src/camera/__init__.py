# src/camera/__init__.py
"""摄像头模块 - 注册所有摄像头类型"""

from src.abstract.camera_factory import CameraFactory
from .usb_camera import USBCamera

CameraFactory.register_camera("usb", USBCamera)