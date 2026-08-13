# src/camera/usb_camera.py
"""USB摄像头实现模块"""

import cv2
import logging
from typing import Dict, Any

from src.abstract.camera_interface import CameraInterface
from src.config_loader import CameraConfig


class USBCamera(CameraInterface):
    """USB摄像头实现"""
    
    def __init__(self, config: Any):
        if isinstance(config, CameraConfig):
            self.device_id = config.device_id
            self.resolution = config.resolution
            self.frame_rate = config.frame_rate
        elif isinstance(config, dict):
            self.device_id = config.get('device_id', 0)
            self.resolution = config.get('resolution', (640, 480))
            self.frame_rate = config.get('frame_rate', 30)
        else:
            self.device_id = 0
            self.resolution = (640, 480)
            self.frame_rate = 30
        
        self.cap = None
        self.logger = logging.getLogger(__name__)
        
        # 尝试自动打开摄像头
        self.open()
    
    def open(self) -> bool:
        """打开摄像头"""
        try:
            self.cap = cv2.VideoCapture(self.device_id)
            if not self.cap.isOpened():
                self.logger.error(f"无法打开摄像头 (设备ID: {self.device_id})")
                return False
            
            # 设置分辨率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            self.logger.info(f"摄像头已打开 (设备ID: {self.device_id}, 分辨率: {self.resolution}, 帧率: {self.frame_rate})")
            return True
        except Exception as e:
            self.logger.error(f"打开摄像头失败: {e}")
            return False
    
    def capture(self, flush_frames: int = 5) -> cv2.Mat:
        """捕获一帧图像"""
        if self.cap is None or not self.cap.isOpened():
            self.logger.warning("摄像头未打开")
            return None

        for _ in range(flush_frames):
            if not self.cap.grab():
                self.logger.warning("grab 失败，可能摄像头掉线或缓冲异常")
                return None

        try:
            ret, frame = self.cap.retrieve()
        except cv2.error as e:
            self.logger.warning(f"retrieve 异常: {e}")
            return None
        if not ret or frame is None:
            self.logger.warning("无法读取帧")
            return None

        return frame
    
    def get_intrinsics(self) -> Dict[str, float]:
        """获取相机内参（简化实现）"""
        return {
            'width': float(self.resolution[0]),
            'height': float(self.resolution[1]),
            'fps': float(self.frame_rate)
        }
    
    def release(self) -> None:
        """释放资源"""
        if self.cap is not None:
            self.cap.release()
            self.logger.info("摄像头资源已释放")
    
    def is_opened(self) -> bool:
        """检查摄像头是否打开"""
        return self.cap is not None and self.cap.isOpened()
