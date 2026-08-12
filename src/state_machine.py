from enum import Enum
from typing import Dict, Any, Optional


class RobotStatus(Enum):
    """机器人状态枚举"""
    SEARCH_TENNIS = "search_tennis"    # 搜索网球
    TRACK_TENNIS = "track_tennis"      # 跟踪网球（差速运动，目标至视野垂直中线且距离达标）
    ALIGN_TENNIS = "align_tennis"      # 对齐网球（低速旋转，目标左x值达标）
    PICK = "pick"                      # 抓取网球
    SEARCH_BUCKET = "search_bucket"    # 寻找桶
    TRACK_BUCKET = "track_bucket"      # 跟踪桶
    PUT_BALL = "put_ball"              # 放球


class StateMachine:
    """状态机类"""
    
    def __init__(self, config: Dict = None) -> None:
        self.current_state: RobotStatus = RobotStatus.SEARCH_TENNIS
        self.previous_state: RobotStatus = RobotStatus.SEARCH_TENNIS
        self.error_message: Optional[str] = None
        self.lost_count = 0
        self.pick_ready_count = 0
        self.config = config or {}
        
        self.target_x = self.config.get('target_x', 0.0)
        self.target_distance = self.config.get('target_distance', 0.3)
        self.threshold_x = self.config.get('threshold_x', 50.0)
        self.threshold_d = self.config.get('threshold_d', 0.05)
        self.grip_threshold = self.config.get('grip_threshold', 90)
        self.grip_close = self.config.get('grip_close', 0)
        self.reach_count_threshold = self.config.get('reach_count_threshold', 10)
        self.frame_width = self.config.get('frame_width', 640)
        self.bucket_edge_threshold = self.config.get('bucket_edge_threshold', 20)

    def get_state(self) -> RobotStatus:
        """获取当前状态"""
        return self.current_state
    
    def set_config(self, config: Dict) -> None:
        """设置配置"""
        self.config = config
        self.target_x = self.config.get('target_x', 0.0)
        self.target_distance = self.config.get('target_distance', 0.3)
        self.threshold_x = self.config.get('threshold_x', 50.0)
        self.threshold_d = self.config.get('threshold_d', 0.05)
        self.grip_threshold = self.config.get('grip_threshold', 90)
        self.grip_close = self.config.get('grip_close', 0)
        self.reach_count_threshold = self.config.get('reach_count_threshold', 10)
        self.frame_width = self.config.get('frame_width', 640)
        self.bucket_edge_threshold = self.config.get('bucket_edge_threshold', 20)

    def transition(self, observation: Dict[str, Any]) -> RobotStatus:
        """根据观测进行状态转换"""
        tennis_detected = observation.get("tennis_detected", False)
        bucket_detected = observation.get("bucket_detected", False)
        tennis_distance = observation.get("tennis_distance", float('inf'))
        tennis_offset_x = observation.get("tennis_offset_x", 0.0)
        tennis_left_edge = observation.get("tennis_left_edge", 0.0)
        bucket_left_edge = observation.get("bucket_left_edge", float('inf'))
        bucket_right_edge = observation.get("bucket_right_edge", -float('inf'))
        gripper_angle = observation.get("gripper_angle", 0)
        
        if self.current_state == RobotStatus.SEARCH_TENNIS:
            if tennis_detected:
                return RobotStatus.TRACK_TENNIS
        
        elif self.current_state == RobotStatus.TRACK_TENNIS:
            if tennis_detected:
                # 目标处于视野垂直中线（容差 2*THRESHOLD_X）且距离达标
                centerline_ok = abs(tennis_offset_x) <= 2 * self.threshold_x
                d_ok = abs(tennis_distance - self.target_distance) <= self.threshold_d

                if centerline_ok and d_ok:
                    self.pick_ready_count += 1
                    if self.pick_ready_count >= self.reach_count_threshold:
                        self.pick_ready_count = 0
                        return RobotStatus.ALIGN_TENNIS
                else:
                    self.pick_ready_count = 0
            else:
                self.pick_ready_count = 0
                self.lost_count += 1
                if self.lost_count >= self.reach_count_threshold:
                    self.lost_count = 0
                    return RobotStatus.SEARCH_TENNIS

        elif self.current_state == RobotStatus.ALIGN_TENNIS:
            if tennis_detected:
                # 目标左x值满足 [TARGET_X, TARGET_X + THRESHOLD_X]
                left_ok = (tennis_left_edge >= self.target_x
                           and tennis_left_edge <= self.target_x + self.threshold_x)

                if left_ok:
                    self.pick_ready_count += 1
                    if self.pick_ready_count >= self.reach_count_threshold:
                        self.pick_ready_count = 0
                        return RobotStatus.PICK
                else:
                    self.pick_ready_count = 0
            else:
                self.pick_ready_count = 0
                self.lost_count += 1
                if self.lost_count >= self.reach_count_threshold:
                    self.lost_count = 0
                    return RobotStatus.SEARCH_TENNIS
        
        elif self.current_state == RobotStatus.PICK:
            if gripper_angle >= self.grip_threshold + self.grip_close:
                return RobotStatus.SEARCH_BUCKET
            else:
                return RobotStatus.SEARCH_TENNIS
        
        elif self.current_state == RobotStatus.SEARCH_BUCKET:
            if bucket_detected:
                return RobotStatus.TRACK_BUCKET
        
        elif self.current_state == RobotStatus.TRACK_BUCKET:
            if bucket_detected:
                left_ok = bucket_left_edge >= 0 and bucket_left_edge <= self.bucket_edge_threshold
                right_ok = bucket_right_edge <= self.frame_width and bucket_right_edge >= self.frame_width - self.bucket_edge_threshold 
                
                if left_ok and right_ok:
                    return RobotStatus.PUT_BALL
            else:
                self.lost_count += 1
                if self.lost_count >= self.reach_count_threshold:
                    self.lost_count = 0
                    return RobotStatus.SEARCH_BUCKET
        
        elif self.current_state == RobotStatus.PUT_BALL:
            return RobotStatus.SEARCH_TENNIS
        
        return self.current_state
    
    def set_state(self, state: RobotStatus) -> None:
        """设置当前状态"""
        self.previous_state = self.current_state
        self.current_state = state
    
    def reset(self) -> None:
        """重置状态机"""
        self.current_state = RobotStatus.SEARCH_TENNIS
        self.previous_state = RobotStatus.SEARCH_TENNIS
        self.error_message = None
        self.lost_count = 0
        self.pick_ready_count = 0