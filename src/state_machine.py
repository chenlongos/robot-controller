# src/state_machine.py
from enum import Enum
from typing import Dict, Any, Optional

class RobotStatus(Enum):
    """机器人状态枚举"""
    INIT = "init"              # 初始化状态
    SEARCH_TENNIS = "search_tennis"    # 搜索网球
    APPROACH_TENNIS = "approach_tennis" # 接近网球（远距离）
    TRACK_TENNIS = "track_tennis"      # 跟踪网球（精细调整）
    PICK = "pick"              # 抓取网球
    SEARCH_BUCKET = "search_bucket"    # 寻找桶
    APPROACH_BUCKET = "approach_bucket" # 接近桶（远距离）
    TRACK_BUCKET = "track_bucket"      # 跟踪桶（精细调整）
    PUT_BALL = "put_ball"      # 放球
    ERROR = "error"            # 错误状态

class StateMachine:
    """状态机类"""
    
    def __init__(self) -> None:
        self.current_state: RobotStatus = RobotStatus.INIT
        self.previous_state: RobotStatus = RobotStatus.INIT
        self.error_message: Optional[str] = None
        self.lost_count = 0  # 目标丢失计数
    
    def get_state(self) -> RobotStatus:
        """获取当前状态"""
        return self.current_state
    
    def transition(self, observation: Dict[str, Any]) -> RobotStatus:
        """根据观测进行状态转换"""
        # 检查是否有错误
        if self._check_error(observation):
            self.set_error("检测到错误")
            return RobotStatus.ERROR
        
        # 状态转换逻辑
        if self.current_state == RobotStatus.INIT:
            if observation.get("initialized", False):
                return RobotStatus.SEARCH_TENNIS
        
        elif self.current_state == RobotStatus.SEARCH_TENNIS:
            if observation.get("tennis_detected", False):
                distance = observation.get("tennis_distance", float('inf'))
                if distance > observation.get("approach_threshold", 0.5):
                    return RobotStatus.APPROACH_TENNIS
                else:
                    return RobotStatus.TRACK_TENNIS
        
        elif self.current_state == RobotStatus.APPROACH_TENNIS:
            if observation.get("tennis_detected", False):
                distance = observation.get("tennis_distance", float('inf'))
                if distance <= observation.get("track_threshold", 0.3):
                    return RobotStatus.TRACK_TENNIS
            else:
                self.lost_count += 1
                if self.lost_count > 10:  # 连续10帧未检测到
                    self.lost_count = 0
                    return RobotStatus.SEARCH_TENNIS
        
        elif self.current_state == RobotStatus.TRACK_TENNIS:
            if observation.get("reached_pick_position", False):
                return RobotStatus.PICK
            elif not observation.get("tennis_detected", False):
                self.lost_count += 1
                if self.lost_count > 10:
                    self.lost_count = 0
                    return RobotStatus.SEARCH_TENNIS
        
        elif self.current_state == RobotStatus.PICK:
            if observation.get("pick_complete", False):
                return RobotStatus.SEARCH_BUCKET
            elif observation.get("pick_failed", False):
                return RobotStatus.SEARCH_TENNIS
        
        elif self.current_state == RobotStatus.SEARCH_BUCKET:
            if observation.get("bucket_detected", False):
                distance = observation.get("bucket_distance", float('inf'))
                if distance > observation.get("approach_threshold", 0.5):
                    return RobotStatus.APPROACH_BUCKET
                else:
                    return RobotStatus.TRACK_BUCKET
        
        elif self.current_state == RobotStatus.APPROACH_BUCKET:
            if observation.get("bucket_detected", False):
                distance = observation.get("bucket_distance", float('inf'))
                if distance <= observation.get("track_threshold", 0.3):
                    return RobotStatus.TRACK_BUCKET
            else:
                self.lost_count += 1
                if self.lost_count > 10:
                    self.lost_count = 0
                    return RobotStatus.SEARCH_BUCKET
        
        elif self.current_state == RobotStatus.TRACK_BUCKET:
            if observation.get("reached_put_position", False):
                return RobotStatus.PUT_BALL
            elif not observation.get("bucket_detected", False):
                self.lost_count += 1
                if self.lost_count > 10:
                    self.lost_count = 0
                    return RobotStatus.SEARCH_BUCKET
        
        elif self.current_state == RobotStatus.PUT_BALL:
            if observation.get("put_complete", False):
                return RobotStatus.SEARCH_TENNIS
            elif observation.get("put_failed", False):
                return RobotStatus.SEARCH_TENNIS
        
        elif self.current_state == RobotStatus.ERROR:
            if observation.get("error_resolved", False):
                return RobotStatus.INIT
        
        return self.current_state
    
    def _check_error(self, observation: Dict[str, Any]) -> bool:
        """检查是否存在错误条件"""
        return observation.get("error", False)
    
    def set_state(self, state: RobotStatus) -> None:
        """设置当前状态"""
        self.previous_state = self.current_state
        self.current_state = state
    
    def set_error(self, message: str) -> None:
        """设置错误状态"""
        self.error_message = message
        self.previous_state = self.current_state
        self.current_state = RobotStatus.ERROR
    
    def reset(self) -> None:
        """重置状态机"""
        self.current_state = RobotStatus.INIT
        self.previous_state = RobotStatus.INIT
        self.error_message = None
        self.lost_count = 0