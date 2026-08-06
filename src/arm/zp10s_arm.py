# src/arm/zp10s_arm.py
"""ZP10S三轴机械臂实现模块"""

import logging
import time
from typing import List, Tuple, Any

from src.abstract.arm_interface import ArmInterface
from src.config_loader import ArmConfig

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    serial = None

logger = logging.getLogger(__name__)


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
        "servo2",
    )
    
    def __init__(self, config: Any):
        """
        :param config: 配置字典或ArmConfig对象，包含以下关键字:
            - port: 串口端口路径 (默认: "/dev/ttyS7")
            - baudrate: 波特率 (默认: 115200)
            - timeout: 超时时间 (默认: 0.1)
        """
        if not HAS_SERIAL:
            raise ImportError("serial模块未安装，请先安装pyserial")
        
        self.config = config
        if isinstance(config, ArmConfig):
            self.port = config.port or '/dev/ttyS7'
            self.baudrate = config.baudrate or 115200
            self.timeout = 0.1
        elif isinstance(config, dict):
            self.port = config.get('port', '/dev/ttyS7')
            self.baudrate = config.get('baudrate', 115200)
            self.timeout = config.get('timeout', 0.1)
        else:
            self.port = '/dev/ttyS7'
            self.baudrate = 115200
            self.timeout = 0.1
        
        self._is_connected = False
        self._is_moving = False
        
        self._last_joint_positions = [180.0, 180.0, 150.0]
        
        self.ser = None
        logger.info("ZP10S Arm initialized")
    
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
        self.restoring_torque()
        logger.info("ZP10S Arm connected")
    
    def _send_frame(self, servo_id: int, angle: float, duration: int = 1000) -> None:
        """发送舵机控制帧"""
        pulse = int(500 + (angle / 270.0) * 2000)
        pulse = max(500, min(2500, pulse))
        cmd = f"#{servo_id:03d}P{pulse:04d}T{duration}!"
        self.ser.write(cmd.encode('ascii'))
        self.ser.flush()
    
    def _send_cmd(self, servo_id: int, cmd: str) -> str:
        """发送指令并返回响应
        
        Args:
            servo_id: 舵机ID，255为广播ID，无返回值
            cmd: 命令字符串
            
        Returns:
            串口返回的响应字符串，广播ID时返回空字符串
        """
        cmd_str = f"#{servo_id:03d}{cmd}!"
        self.ser.write(cmd_str.encode('ascii'))
        self.ser.flush()
        
        if servo_id == 255:
            return ""
        
        time.sleep(0.02)
        
        response = ""
        while self.ser.in_waiting > 0:
            response += self.ser.read(self.ser.in_waiting).decode('ascii', errors='ignore')
        
        return response
    
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
            positions: 三个关节的目标位置列表 [servo0, servo1, servo2]
        """
        if not self.is_connected:
            raise RuntimeError("ZP10S Arm not connected")
        
        if len(positions) != len(self.JOINT_NAMES):
            raise ValueError(f"Expected {len(self.JOINT_NAMES)} joint positions")
        
        self._is_moving = True
        
        for i, pos in enumerate(positions):
            self.set_angle(i, pos)
        
        time.sleep(0.05)
        self._is_moving = False
    
    def get_joint_positions(self) -> List[float]:
        """获取当前所有关节位置
        
        通过发送PRAD命令读取每个舵机的PWM值，然后转换为角度。
        
        Returns:
            三个关节的位置列表 [servo0, servo1, servo2]，单位为角度
        """
        if not self.is_connected:
            raise RuntimeError("ZP10S Arm not connected")
        
        positions = []
        for servo_id in range(3):
            response = self._send_cmd(servo_id, "PRAD")
            
            pwm_value = 1500
            if response:
                parts = response.strip().split('P')
                if len(parts) >= 2:
                    pwm_part = parts[1].split('!')[0]
                    try:
                        pwm_value = int(pwm_part)
                    except ValueError:
                        pass
            
            angle = ((pwm_value - 500) / 2000.0) * 270.0
            angle = max(0.0, min(270.0, angle))
            positions.append(angle)
        
        self._last_joint_positions = positions
        return positions.copy()
    
    def get_gripper_position(self) -> float:
        """获取夹爪位置
        
        Returns:
            夹爪位置 (0-100)，基于servo2的角度计算
        """
        if not self.is_connected:
            raise RuntimeError("ZP10S Arm not connected")
        
        positions = self.get_joint_positions()
        servo2_angle = positions[2]
        return servo2_angle
    
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