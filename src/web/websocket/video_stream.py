# src/web/websocket/video_stream.py
import cv2
import base64
from fastapi import WebSocket
from typing import List

class VideoStreamManager:
    """视频流管理器"""
    
    def __init__(self) -> None:
        self.active_connections: List[WebSocket] = []
        self.running: bool = False
        self.camera = None
    
    async def connect(self, websocket: WebSocket) -> None:
        """接受新连接"""
        pass
    
    def disconnect(self, websocket: WebSocket) -> None:
        """断开连接"""
        pass
    
    async def start_streaming(self) -> None:
        """持续推送视频帧"""
        pass