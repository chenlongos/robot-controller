# src/controller/arm_controller.py
"""机械臂控制器模块"""

import json
import logging
import time
from pathlib import Path
from typing import List, Dict, Any

from src.abstract.arm_interface import ArmInterface


logger = logging.getLogger(__name__)


class ArmController:
    """机械臂控制器"""
    
    ALLOWED_ACTIONS = {"origin", "pick", "put", "open_gripper", "close_gripper"}
    ALLOWED_STEP_TYPES = {"joint", "gripper", "delay"}
    
    DEFAULT_ACTION_SEQUENCES = {
        "origin": [],
        "pick": [],
        "put": [],
        "open_gripper": [],
        "close_gripper": []
    }
    
    def __init__(self, arm: ArmInterface, robot_name: str = "aka00v4-rk3576", step_delay: float = 0.5) -> None:
        self.arm = arm
        self.robot_name = robot_name
        self.step_delay = step_delay
    
    def _get_config_path(self) -> Path:
        """获取配置文件路径，文件名包含机器人名字"""
        return Path(__file__).resolve().parents[2] / f"config/arm_action_sequences_{self.robot_name}.json"
    
    def _load_action_sequences(self) -> Dict[str, List[Dict]]:
        """从配置文件加载动作序列"""
        config_path = self._get_config_path()
        
        if not config_path.exists():
            logger.warning(f"配置文件不存在，使用默认动作序列: {config_path}")
            return self.DEFAULT_ACTION_SEQUENCES.copy()
        
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            sequences = self.DEFAULT_ACTION_SEQUENCES.copy()
            for action_name in self.ALLOWED_ACTIONS:
                if action_name in data and isinstance(data[action_name], list):
                    sequences[action_name] = data[action_name]
            
            logger.info(f"动作序列加载成功: {config_path}")
            return sequences
        
        except Exception as e:
            logger.error(f"加载动作序列失败: {e}")
            return self.DEFAULT_ACTION_SEQUENCES.copy()
    
    def _save_action_sequences(self, sequences: Dict[str, List[Dict]]) -> None:
        """保存动作序列到配置文件"""
        config_path = self._get_config_path()
        
        try:
            config_path.parent.mkdir(parents=True, exist_ok=True)
            
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump(sequences, f, indent=2, ensure_ascii=False)
            
            logger.info(f"动作序列保存成功: {config_path}")
        
        except Exception as e:
            logger.error(f"保存动作序列失败: {e}")
            raise
    
    def set_action_sequence(self, action_name: str, sequence: List[Dict]) -> None:
        """设置动作序列并保存到配置文件
        
        Args:
            action_name: 动作名称，可选值: origin, pick, put, open_gripper, close_gripper
            sequence: 动作序列，每个元素为一个步骤字典
                      步骤类型: {"type": "joint", "positions": [x1, x2, ...]}
                               {"type": "gripper", "position": 0~100}
        
        Raises:
            ValueError: 如果动作名称不合法
        """
        if action_name not in self.ALLOWED_ACTIONS:
            raise ValueError(f"不支持的动作名称: {action_name}，可选值: {self.ALLOWED_ACTIONS}")
        
        for step in sequence:
            step_type = step.get("type")
            if step_type not in self.ALLOWED_STEP_TYPES:
                raise ValueError(f"不支持的步骤类型: {step_type}，可选值: {self.ALLOWED_STEP_TYPES}")
            
            if step_type == "joint":
                if "positions" not in step or not isinstance(step["positions"], list):
                    raise ValueError("joint类型步骤必须包含positions列表")
            elif step_type == "gripper":
                if "position" not in step:
                    raise ValueError("gripper类型步骤必须包含position字段")
            elif step_type == "delay":
                if "duration" not in step or not isinstance(step["duration"], (int, float)):
                    raise ValueError("delay类型步骤必须包含duration字段")
        
        sequences = self._load_action_sequences()
        sequences[action_name] = sequence
        self._save_action_sequences(sequences)
        
        logger.info(f"动作序列 '{action_name}' 设置成功")
    
    def execute_action_sequence(self, action_name: str) -> Dict[str, Any]:
        """从配置文件读取并执行动作序列
        
        Args:
            action_name: 动作名称，可选值: origin, pick, put, open_gripper, close_gripper
        
        Returns:
            执行结果，包含成功状态和执行时间等信息
        
        Raises:
            ValueError: 如果动作名称不合法
        """
        if action_name not in self.ALLOWED_ACTIONS:
            raise ValueError(f"不支持的动作名称: {action_name}，可选值: {self.ALLOWED_ACTIONS}")
        
        sequences = self._load_action_sequences()
        sequence = sequences.get(action_name, [])
        
        if not sequence:
            logger.warning(f"动作序列 '{action_name}' 为空")
            return {"status": "completed", "action_name": action_name, "steps": 0, "duration": 0}
        
        start_time = time.time()
        step_count = 0
        
        logger.info(f"开始执行动作序列: {action_name}")
        
        try:
            for step in sequence:
                step_type = step.get("type")
                
                if step_type == "joint":
                    positions = step.get("positions", [])
                    current_positions = list(self.arm.get_joint_positions())
                    for i, pos in enumerate(positions):
                        if i < len(current_positions):
                            current_positions[i] = pos
                    logger.debug(f"移动关节位置: {current_positions}")
                    self.arm.move_to_joint_positions(current_positions)
                
                elif step_type == "gripper":
                    servo2_angle = step.get("position", 0)
                    logger.debug(f"设置夹爪位置: {servo2_angle}")
                    
                    positions = self.arm.get_joint_positions()
                    positions[2] = servo2_angle
                    logger.debug(f"转换为关节位置: {positions}")
                    self.arm.move_to_joint_positions(positions)
                
                elif step_type == "delay":
                    duration = step.get("duration", 0)
                    logger.debug(f"延时等待: {duration}s")
                    time.sleep(duration)
                
                step_count += 1
                
                if step_type != "delay":
                    time.sleep(self.step_delay)
            
            duration = time.time() - start_time
            logger.info(f"动作序列 '{action_name}' 执行完成，共 {step_count} 步，耗时 {duration:.2f}s")
            
            return {
                "status": "completed",
                "action_name": action_name,
                "steps": step_count,
                "duration": round(duration, 2)
            }
        
        except Exception as e:
            logger.error(f"执行动作序列 '{action_name}' 失败: {e}")
            return {
                "status": "failed",
                "action_name": action_name,
                "error": str(e),
                "steps_completed": step_count
            }
    
    def stop(self) -> None:
        """停止机械臂"""
        logger.info("停止机械臂")
        self.arm.stop()