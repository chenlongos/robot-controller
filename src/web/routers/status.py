# src/web/routers/status.py
from fastapi import APIRouter

router: APIRouter = APIRouter()

@router.get("/")
async def get_status():
    """获取机器人完整状态"""
    pass

@router.get("/arm")
async def get_arm_status():
    """获取机械臂状态"""
    pass

@router.get("/base")
async def get_base_status():
    """获取底盘状态"""
    pass

@router.get("/camera")
async def get_camera_status():
    """获取摄像头状态"""
    pass