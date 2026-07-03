# src/types.py
from typing import TypedDict, List, Dict, Optional
import numpy as np

class Observation(TypedDict):
    """机器人观测数据格式
    
    用于描述机器人在某一时刻的状态信息，包含传感器数据和运动状态。
    """
    camera: np.ndarray              # 摄像头图像
    joint_positions: List[float]    # 关节位置
    gripper_position: float         # 夹爪位置 (0-1, 0为闭合)
    base_velocity: Dict[str, float] # 底盘速度 {"x": x_vel, "y": y_vel, "theta": theta_vel}
    timestamp: float                # 时间戳 (秒)

class BaseAction(TypedDict):
    """底盘动作指令格式"""
    x_vel: float    # 前进速度 (m/s)
    y_vel: float    # 横向速度 (m/s)
    theta_vel: float # 旋转速度 (rad/s)

class ArmAction(TypedDict):
    """机械臂动作指令格式"""
    joints: List[float]  # 关节目标位置
    gripper: float       # 夹爪目标位置 (0-1)

class Action(TypedDict):
    """机器人动作指令格式
    
    用于描述机器人的运动指令，包含底盘和机械臂两部分。
    """
    base: Optional[BaseAction]  # 底盘动作（可选）
    arm: Optional[ArmAction]    # 机械臂动作（可选）