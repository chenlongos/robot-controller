# src/arm/zp10s_arm.py
"""ZP10S三轴机械臂实现模块"""

import logging
import time
import json
from pathlib import Path
from typing import List, Dict, Tuple

from src.abstract.arm_interface import ArmInterface

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    serial = None

logger = logging.getLogger(__name__)

DEFAULT_ZP10S_ARM_ANGLES = {
    "servo0_prepare": 245,
    "servo1_prepare": 180,
    "servo2_prepare": 150,
    "servo2_approach": 150,
    "servo2_grab": 90,
    "servo0_lift": 200,
    "servo1_lift": 180,
    "servo2_lift": 90,
}


class ZP10SArm(ArmInterface):
    """ZP10S三轴机械臂实现
    
    使用串口控制三关节机械臂，包含：
    - servo0: 底座旋转
    - servo1: 手臂升降
    - servo2: 夹爪
    
    硬件层仅提供关节级控制，抓取动作等高级控制由上层实现。
    """
    
    JOINT_NAMES: Tuple[str, ...] = (
        "servo0",
        "servo1",
    )
    
    def __init__(self, config: Dict):
        """
        :param config: 配置字典，包含以下关键字:
            - port: 串口端口路径 (默认: "/dev/ttyS2")
            - baudrate: 波特率 (默认: 115200)
            - timeout: 超时时间 (默认: 0.1)
        """
        if not HAS_SERIAL:
            raise ImportError("serial模块未安装，请先安装pyserial")
        
        self.config = config
        self.port = config.get('port', '/dev/ttyS7')
        self.baudrate = config.get('baudrate', 115200)
        self.timeout = config.get('timeout', 0.1)
        
        self._angles = self._load_arm_angles()
        self._is_connected = False
        self._is_moving = False
        
        self._last_joint_positions = [180.0, 180.0]
        self._last_gripper_position = 50.0
        
        self.ser = None
        logger.info("ZP10S Arm initialized")
    
    def _load_arm_angles(self) -> Dict:
        arm_angles_path = Path(__file__).resolve().parents[2] / "arm_angles.json"
        if arm_angles_path.exists():
            try:
                data = json.loads(arm_angles_path.read_text(encoding="utf-8"))
                if isinstance(data, dict):
                    return {**DEFAULT_ZP10S_ARM_ANGLES, **data}
            except Exception:
                pass
        return DEFAULT_ZP10S_ARM_ANGLES.copy()
    
    @property
    def is_connected(self) -> bool:
        """检查机械臂是否已连接"""
        return self._is_connected and self.ser is not None and self.ser.is_open
    
    def connect(self, calibrate: bool = True) -> None:
        """连接机械臂
        
        Args:
            calibrate: 是否执行校准 (默认: True，ZP10S不使用校准)
        """
        if self.is_connected:
            logger.warning("ZP10S Arm already connected")
            return
        
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout
        )
        
        self._is_connected = True
        logger.info("ZP10S Arm connected")
    
    def _angle(self, key: str, default: int) -> int:
        return self._angles.get(key, default)
    
    @property
    def id2_angle_open(self) -> int:
        return self._angle("servo2_prepare", 150)
    
    @property
    def id2_angle_close(self) -> int:
        return self._angle("servo2_grab", 90)
    
    def _send_frame(self, servo_id: int, angle: float, duration: int = 1000) -> None:
        """发送舵机控制帧"""
        pulse = int(500 + (angle / 270.0) * 2000)
        pulse = max(500, min(2500, pulse))
        cmd = f"#{servo_id:03d}P{pulse:04d}T{duration}!"
        self.ser.write(cmd.encode('ascii'))
        self.ser.flush()
    
    def _send_cmd(self, servo_id: int, cmd: str) -> None:
        """发送指令"""
        cmd_str = f"#{servo_id:03d}{cmd}!"
        self.ser.write(cmd_str.encode('ascii'))
        self.ser.flush()
    
    def release_torque(self) -> None:
        """释放扭矩"""
        self._send_cmd(255, "PULK")
    
    def restoring_torque(self) -> None:
        """恢复扭矩"""
        self._send_cmd(255, "PULR")
    
    def set_angle(self, servo_id: int, angle: float) -> None:
        """设置单个舵机角度
        
        Args:
            servo_id: 舵机ID (0-2)
            angle: 角度 (0-270)
        """
        if not self.is_connected:
            raise RuntimeError("ZP10S Arm not connected")
        
        if not 0 <= angle <= 270:
            raise ValueError("angle must be 0~270")
        
        self._send_frame(servo_id, angle)
    
    def move_to_joint_positions(self, positions: List[float]) -> None:
        """移动到关节位置
        
        Args:
            positions: 两个关节的目标位置列表 [servo0, servo1]
        """
        if not self.is_connected:
            raise RuntimeError("ZP10S Arm not connected")
        
        if len(positions) != len(self.JOINT_NAMES):
            raise ValueError(f"Expected {len(self.JOINT_NAMES)} joint positions")
        
        self._is_moving = True
        
        for i, pos in enumerate(positions):
            self.set_angle(i, pos)
        
        self._last_joint_positions = list(positions)
        
        time.sleep(0.05)
        self._is_moving = False
    
    def get_joint_positions(self) -> List[float]:
        """获取当前关节位置
        
        Returns:
            两个关节的位置列表
        """
        if not self.is_connected:
            raise RuntimeError("ZP10S Arm not connected")
        
        return self._last_joint_positions.copy()
    
    def set_gripper_position(self, position: float) -> None:
        """设置夹爪位置
        
        Args:
            position: 夹爪位置 (0-100, 0为闭合，100为张开)
        """
        if not self.is_connected:
            raise RuntimeError("ZP10S Arm not connected")
        
        clamped_position = max(0.0, min(100.0, position))
        self._last_gripper_position = clamped_position
        
        if clamped_position >= 50:
            angle = self.id2_angle_open
        else:
            angle = self.id2_angle_close
        
        self.set_angle(2, angle)
    
    def get_gripper_position(self) -> float:
        """获取夹爪位置
        
        Returns:
            夹爪位置 (0-100)
        """
        if not self.is_connected:
            raise RuntimeError("ZP10S Arm not connected")
        
        return self._last_gripper_position
    
    def is_moving(self) -> bool:
        """检查是否正在移动"""
        return self._is_moving
    
    def stop(self) -> None:
        """停止运动"""
        self._is_moving = False
        logger.info("ZP10S Arm stopped")
    
    def disconnect(self) -> None:
        """断开连接"""
        if not self.is_connected:
            logger.warning("ZP10S Arm not connected")
            return
        
        self.stop()
        
        self.release_torque()
        
        if self.ser.is_open:
            self.ser.close()
        
        self._is_connected = False
        logger.info("ZP10S Arm disconnected")