# src/web/routers/config.py
from fastapi import APIRouter, HTTPException
from typing import Dict, List

router: APIRouter = APIRouter()

@router.get("/")
async def get_current_config():
    """获取当前配置"""
    pass

@router.put("/")
async def update_config(config_updates: Dict):
    """更新配置（运行时生效）"""
    pass

@router.post("/save")
async def save_config(request: Dict):
    """保存配置为新文件"""
    pass

@router.get("/list")
async def list_configs():
    """列出所有配置文件"""
    pass

@router.get("/{name}")
async def get_config(name: str):
    """获取指定配置文件"""
    pass