# src/web/server.py
from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles
from src.web.routers import status, control, config
from src.web.websocket.video_stream import VideoStreamManager

app: FastAPI = FastAPI(title="机器人远程控制中心")
video_manager: VideoStreamManager = VideoStreamManager()

def setup_routes() -> None:
    """设置路由"""
    pass

def setup_static_files() -> None:
    """设置静态文件服务"""
    pass

def start_server(host: str = "0.0.0.0", port: int = 8000) -> None:
    """启动Web服务器"""
    pass