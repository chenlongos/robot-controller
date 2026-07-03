# src/controller/arm_controller.py
"""机械臂控制器模块"""

from typing import List, Dict, Any
from src.abstract.arm_interface import ArmInterface


class ArmController:
    """机械臂控制器"""
    
    def __init__(self, arm: ArmInterface) -> None:
        self.arm: ArmInterface = arm
        self.pick_sequence: List[Dict] = []
        self.put_sequence: List[Dict] = []
    
    def set_pick_sequence(self, sequence: List[Dict]) -> None:
        """设置抓取动作序列"""
        self.pick_sequence = sequence
    
    def set_put_sequence(self, sequence: List[Dict]) -> None:
        """设置放置动作序列"""
        self.put_sequence = sequence
    
    def execute_pick_sequence(self) -> Dict[str, Any]:
        """执行抓取序列"""
        for action in self.pick_sequence:
            if "joints" in action:
                self.arm.move_to_joint_positions(action["joints"])
            if "gripper" in action:
                self.arm.set_gripper_position(action["gripper"])
        return {"status": "completed"}
    
    def execute_put_sequence(self, sequence: List[Dict] = None) -> Dict[str, Any]:
        """执行放置序列"""
        target_sequence = sequence if sequence else self.put_sequence
        for action in target_sequence:
            if "joints" in action:
                self.arm.move_to_joint_positions(action["joints"])
            if "gripper" in action:
                self.arm.set_gripper_position(action["gripper"])
        return {"status": "completed"}
    
    def move_to_position(self, x: float, y: float, z: float) -> None:
        """移动到笛卡尔坐标"""
        self.arm.move_to_cartesian(x, y, z)
    
    def open_gripper(self) -> None:
        """打开夹爪"""
        self.arm.set_gripper_position(1.0)
    
    def close_gripper(self) -> None:
        """关闭夹爪"""
        self.arm.set_gripper_position(0.0)
    
    def stop(self) -> None:
        """停止机械臂"""
        self.arm.stop()
