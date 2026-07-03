# src/arm/so101_arm.py
"""SO101六轴机械臂实现模块"""

import logging
import time
from typing import List, Dict

from src.abstract.arm_interface import ArmInterface

try:
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
    HAS_LEROBOT = True
except ImportError:
    HAS_LEROBOT = False
    Motor = None
    MotorCalibration = None
    MotorNormMode = None
    FeetechMotorsBus = None
    OperatingMode = None

logger = logging.getLogger(__name__)


class SO101Arm(ArmInterface):
    """SO101六轴机械臂实现
    
    使用Feetech电机总线控制六关节机械臂，包含：
    - shoulder_pan: 肩部旋转
    - shoulder_lift: 肩部升降
    - elbow_flex: 肘部弯曲
    - wrist_flex: 腕部弯曲
    - wrist_roll: 腕部旋转
    - gripper: 夹爪
    """
    
    JOINT_NAMES = [
        "shoulder_pan",
        "shoulder_lift", 
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper"
    ]
    
    def __init__(self, config: Dict):
        """
        :param config: 配置字典，包含以下关键字:
            - port: 串口端口路径 (默认: "/dev/ttyUSB0")
            - use_degrees: 是否使用角度模式 (默认: False)
            - disable_torque_on_disconnect: 断开连接时是否禁用扭矩 (默认: True)
            - max_relative_target: 最大相对目标位置 (可选)
            - calibration: 校准数据 (可选)
        """
        if not HAS_LEROBOT:
            raise ImportError("lerobot模块未安装，请先安装lerobot")
        
        self.config = config
        self.port = config.get('port', '/dev/ttyUSB0')
        self.use_degrees = config.get('use_degrees', False)
        self.disable_torque_on_disconnect = config.get('disable_torque_on_disconnect', True)
        self.max_relative_target = config.get('max_relative_target', None)
        self.calibration_data = config.get('calibration', None)
        
        norm_mode_body = MotorNormMode.DEGREES if self.use_degrees else MotorNormMode.RANGE_M100_100
        
        self.bus = FeetechMotorsBus(
            port=self.port,
            motors={
                "shoulder_pan": Motor(id=1, motor_type="sts3215", norm_mode=norm_mode_body),
                "shoulder_lift": Motor(id=2, motor_type="sts3215", norm_mode=norm_mode_body),
                "elbow_flex": Motor(id=3, motor_type="sts3215", norm_mode=norm_mode_body),
                "wrist_flex": Motor(id=4, motor_type="sts3215", norm_mode=norm_mode_body),
                "wrist_roll": Motor(id=5, motor_type="sts3215", norm_mode=norm_mode_body),
                "gripper": Motor(id=6, motor_type="sts3215", norm_mode=MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration_data,
        )
        
        self._is_connected = False
        self._is_moving = False
        logger.info("SO101 Arm initialized")
    
    @property
    def is_connected(self) -> bool:
        """检查机械臂是否已连接"""
        return self._is_connected and self.bus.is_connected
    
    def connect(self, calibrate: bool = True) -> None:
        """连接机械臂"""
        if self.is_connected:
            logger.warning("SO101 Arm already connected")
            return
        
        self.bus.connect()
        
        if not self.is_calibrated and calibrate:
            logger.info("Running calibration...")
            self.calibrate()
        
        self.configure()
        self._is_connected = True
        logger.info("SO101 Arm connected")
    
    @property
    def is_calibrated(self) -> bool:
        """检查是否已校准"""
        return self.bus.is_calibrated
    
    def calibrate(self) -> None:
        """执行校准"""
        if self.calibration_data:
            logger.info("Using provided calibration data")
            self.bus.write_calibration(self.calibration_data)
            return
        
        logger.info("Running manual calibration...")
        self.bus.disable_torque()
        
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
        
        input("Move arm to the middle of its range of motion and press ENTER...")
        homing_offsets = self.bus.set_half_turn_homings()
        
        print("Move all joints sequentially through their entire ranges of motion.")
        print("Recording positions. Press ENTER to stop...")
        range_mins, range_maxes = self.bus.record_ranges_of_motion()
        
        self.calibration_data = {}
        for motor, m in self.bus.motors.items():
            self.calibration_data[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )
        
        self.bus.write_calibration(self.calibration_data)
        logger.info("Calibration completed")
    
    def configure(self) -> None:
        """配置电机参数"""
        with self.bus.torque_disabled():
            self.bus.configure_motors()
            for motor in self.bus.motors:
                self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                self.bus.write("P_Coefficient", motor, 16)
                self.bus.write("I_Coefficient", motor, 0)
                self.bus.write("D_Coefficient", motor, 32)
                
                if motor == "gripper":
                    self.bus.write("Max_Torque_Limit", motor, 500)
                    self.bus.write("Protection_Current", motor, 250)
                    self.bus.write("Overload_Torque", motor, 25)
        
        logger.info("SO101 Arm configured")
    
    def move_to_joint_positions(self, positions: List[float]) -> None:
        """移动到关节位置
        
        :param positions: 六个关节的目标位置列表
                         [shoulder_pan, shoulder_lift, elbow_flex, 
                          wrist_flex, wrist_roll, gripper]
        """
        if not self.is_connected:
            raise RuntimeError("SO101 Arm not connected")
        
        if len(positions) != 6:
            raise ValueError("Expected 6 joint positions")
        
        goal_pos = dict(zip(self.JOINT_NAMES, positions))
        
        if self.max_relative_target is not None:
            present_pos = self.bus.sync_read("Present_Position")
            for joint, g_pos in goal_pos.items():
                if joint in present_pos:
                    diff = abs(g_pos - present_pos[joint])
                    if diff > self.max_relative_target:
                        direction = 1 if g_pos > present_pos[joint] else -1
                        goal_pos[joint] = present_pos[joint] + direction * self.max_relative_target
        
        self._is_moving = True
        self.bus.sync_write("Goal_Position", goal_pos)
        
        time.sleep(0.05)
        self._is_moving = False
    
    def move_to_cartesian(self, x: float, y: float, z: float) -> None:
        """移动到笛卡尔坐标
        
        注意：此方法需要逆运动学求解，当前实现为占位符
        
        :param x: X坐标
        :param y: Y坐标
        :param z: Z坐标
        """
        if not self.is_connected:
            raise RuntimeError("SO101 Arm not connected")
        
        logger.warning("move_to_cartesian requires inverse kinematics, "
                      "which is not yet implemented. Using default position.")
        
        default_positions = [0.0, -30.0, 90.0, -60.0, 0.0, 50.0] if self.use_degrees else [0.0, -0.3, 0.5, -0.3, 0.0, 50.0]
        self.move_to_joint_positions(default_positions)
    
    def get_joint_positions(self) -> List[float]:
        """获取当前关节位置"""
        if not self.is_connected:
            raise RuntimeError("SO101 Arm not connected")
        
        positions = self.bus.sync_read("Present_Position")
        return [positions.get(joint, 0.0) for joint in self.JOINT_NAMES[:-1]]
    
    def set_gripper_position(self, position: float) -> None:
        """设置夹爪位置
        
        :param position: 夹爪位置 (0-100, 0为闭合，100为张开)
        """
        if not self.is_connected:
            raise RuntimeError("SO101 Arm not connected")
        
        clamped_position = max(0.0, min(100.0, position))
        self.bus.sync_write("Goal_Position", {"gripper": clamped_position})
    
    def get_gripper_position(self) -> float:
        """获取夹爪位置"""
        if not self.is_connected:
            raise RuntimeError("SO101 Arm not connected")
        
        positions = self.bus.sync_read("Present_Position")
        return positions.get("gripper", 0.0)
    
    def is_moving(self) -> bool:
        """检查是否正在移动"""
        return self._is_moving
    
    def stop(self) -> None:
        """停止运动"""
        self._is_moving = False
        present_pos = self.bus.sync_read("Present_Position")
        self.bus.sync_write("Goal_Position", present_pos)
        logger.info("SO101 Arm stopped")
    
    def disconnect(self) -> None:
        """断开连接"""
        if not self.is_connected:
            logger.warning("SO101 Arm not connected")
            return
        
        self.stop()
        self.bus.disconnect(self.disable_torque_on_disconnect)
        self._is_connected = False
        logger.info("SO101 Arm disconnected")