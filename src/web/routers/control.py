# src/web/routers/control.py
from fastapi import APIRouter
from typing import Dict

router: APIRouter = APIRouter()

@router.post("/start")
async def start_robot(command: Dict[str, str]):
    """启动机器人"""
    pass

@router.post("/stop")
async def stop_robot():
    """停止机器人"""
    pass

@router.post("/emergency_stop")
async def emergency_stop():
    """紧急停止"""
    pass
