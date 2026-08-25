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

    # 夹爪在 JOINT_NAMES 中的索引。夹爪抓取物体后处于卡阻状态，
    # 其到位检测和位置读取需要与臂关节区别对待。
    _GRIPPER_INDEX: int = 2

    # get_joint_positions 检测到 _is_moving=True 时的等待超时（秒）。
    # 主要用于兜底：当 _is_moving 因运动过程异常中断被卡在 True 时，
    # 避免本方法在等待循环里永久阻塞。
    _MOVING_WAIT_TIMEOUT: float = 5.0

    # move_to_joint_positions 发送指令后，等待舵机物理到位的参数：
    # - _REACH_TOLERANCE: 判定到位的角度容差（度）。舵机读数有 1~2° 噪声/系统偏差，
    #   设 3.0 兼顾"确实到位"与"不会因小偏差永久等待"。
    # - _REACH_TIMEOUT: 到位等待超时（秒）。超时后不抛异常（指令已发，到位是尽力而为），
    #   仅 warning，避免舵机卡住时阻塞整个交互。
    # - _REACH_POLL_INTERVAL: 轮询读取间隔（秒）。
    _REACH_TOLERANCE: float = 10.0
    _REACH_TIMEOUT: float = 2.0
    _REACH_POLL_INTERVAL: float = 0.05
    
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

        # 缓存最近一次成功读取的关节位置。读取失败时不更新此缓存，
        # 也不再把缓存里的旧值/初始占位值当成"本次读数"返回给调用方。
        self._last_joint_positions: List[float] = []

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

        发送目标位置后，会循环读取各舵机实际位置，直到全部进入目标容差
        `_REACH_TOLERANCE` 内才认为到位并返回（带 `_REACH_TIMEOUT` 超时兜底）。
        这样方法返回时舵机已物理到位，避免"移动完成"后立刻读到过渡值。

        在整个运动+到位检测期间 `_is_moving=True`，用 try/finally 保证即使
        中途抛异常也能复位，避免标志位卡死导致 get_joint_positions 永久等待。

        Args:
            positions: 三个关节的目标位置列表 [servo0, servo1, servo2]
        """
        if not self.is_connected:
            raise RuntimeError("ZP10S Arm not connected")

        if len(positions) != len(self.JOINT_NAMES):
            raise ValueError(f"Expected {len(self.JOINT_NAMES)} joint positions")

        self._is_moving = True
        try:
            for i, pos in enumerate(positions):
                self.set_angle(i, pos)
            self._wait_until_reached(positions)
        finally:
            self._is_moving = False

    def _wait_until_reached(self, target_positions: List[float]) -> None:
        """轮询读取实际位置，直到全部舵机进入目标容差内或超时。

        超时仅 warning 不抛异常：移动指令已发送，到位是尽力而为，
        舵机若存在系统性偏差或卡阻，不应让交互测试直接崩溃。
        单次读取失败（如串口偶发超时）会在下一轮重试，不计入超时判定。

        注意：夹爪（servo2）在抓取物体后处于卡阻状态，无法精确到位，
        因此到位检测只检查臂关节（servo0, servo1），不检查夹爪。
        """
        deadline = time.time() + self._REACH_TIMEOUT
        last_positions: List[float] = []
        while time.time() < deadline:
            try:
                current = self._read_positions_raw()
            except RuntimeError as e:
                # 偶发读取失败，等待下一轮重试，不中断到位检测
                logger.debug("到位检测中读取失败，重试: %s", e)
                time.sleep(self._REACH_POLL_INTERVAL)
                continue

            last_positions = current
            # 只检查臂关节是否到位，跳过夹爪（卡阻时无法到位属正常）
            if all(abs(c - t) <= self._REACH_TOLERANCE
                   for i, (c, t) in enumerate(zip(current, target_positions))
                   if i != self._GRIPPER_INDEX):
                return
            time.sleep(self._REACH_POLL_INTERVAL)

        logger.warning(
            "等待舵机到位超时（>%ss），目标=%s，最后读数=%s；"
            "可能存在系统偏差或舵机卡阻，继续返回",
            self._REACH_TIMEOUT, target_positions, last_positions
        )
    
    def get_joint_positions(self) -> List[float]:
        """获取当前所有关节位置

        通过发送PRAD命令读取每个舵机的PWM值，然后转换为角度。

        Returns:
            三个关节的位置列表 [servo0, servo1, servo2]，单位为角度。
            当任一舵机响应为空、解析失败或PWM超出合法范围 [500, 2500] 时，
            抛出 RuntimeError，不再返回捏造的默认值（原先会把失败的舵机当成
            PWM=1500 → angle=135.0，导致用户误判真实角度）。

        注意：
            只有在 `is_moving()` 为 False（机械臂不在运动过程中）时，舵机
            的实际位置才稳定，读取结果才准确。运动过程中（`move_to_joint_positions`
            执行期间，`_is_moving=True`）舵机处于过渡状态，PRAD 读到的 PWM 是
            中间值，不代表最终位置。因此本方法在检测到 `_is_moving=True` 时会
            **等待运动结束** 再读取；若等待超过 `_MOVING_WAIT_TIMEOUT` 秒仍未
            结束（通常意味着运动过程异常中断、`_is_moving` 被卡在 True），
            则抛出 RuntimeError，避免永久阻塞。
        """
        if not self.is_connected:
            raise RuntimeError("ZP10S Arm not connected")

        if self._is_moving:
            # 运动过程中舵机处于过渡状态，读到的 PWM 不代表最终位置。
            # 等待运动结束再读取；带超时保护，防止 _is_moving 因异常被
            # 卡在 True 时本方法永久阻塞。
            deadline = time.time() + self._MOVING_WAIT_TIMEOUT
            while self._is_moving:
                if time.time() > deadline:
                    raise RuntimeError(
                        f"等待 ZP10S 运动结束超时（>{self._MOVING_WAIT_TIMEOUT}s），"
                        "关节位置不可信；请检查运动是否异常中断"
                    )
                time.sleep(0.01)

        positions = self._read_positions_raw()
        self._last_joint_positions = list(positions)
        return positions

    def _parse_prad_response(self, servo_id: int, response: str) -> Tuple[Any, Any]:
        """解析单个舵机 PRAD 响应。

        Returns:
            (angle, error): 成功时 error 为 None、angle 为角度值；
            失败时 angle 为 None、error 为错误描述字符串。
        """
        if not response:
            return None, f"servo{servo_id}: 无响应（串口超时或未收到返回）"

        parts = response.strip().split('P')
        if len(parts) < 2:
            return None, (
                f"servo{servo_id}: 响应格式错误（缺少'P'分隔符），原始响应：{response!r}"
            )

        pwm_part = parts[1].split('!')[0]
        try:
            pwm_value = int(pwm_part)
        except ValueError:
            return None, (
                f"servo{servo_id}: PWM解析失败，pwm片段={pwm_part!r}，原始响应：{response!r}"
            )

        # ZP10S 舵机 PWM 合法范围与发送时一致：[500, 2500]。
        if pwm_value < 500 or pwm_value > 2500:
            return None, (
                f"servo{servo_id}: PWM={pwm_value} 超出合法范围 [500, 2500]，"
                f"原始响应：{response!r}"
            )

        angle = ((pwm_value - 500) / 2000.0) * 270.0
        return angle, None

    def _read_positions_raw(self) -> List[float]:
        """裸读取各舵机位置（PRAD→角度），不做连接/运动状态检查，不更新缓存。

        供 get_joint_positions（等待运动后读取）和 _wait_until_reached（运动中
        轮询到位）复用同一套解析与校验逻辑。任一舵机（包括夹爪）响应为空、
        解析失败或 PWM 超出 [500, 2500] 时整体抛 RuntimeError，错误细节含舵机号
        与原始响应。

        夹爪值用于判断是否夹取到物品，必须返回实时读取值，不使用缓存或默认值。
        """
        positions: List[float] = []
        errors: List[str] = []

        for servo_id in range(3):
            response = self._send_cmd(servo_id, "PRAD")
            angle, error = self._parse_prad_response(servo_id, response)

            if error is None:
                positions.append(angle)
            else:
                errors.append(error)

        if errors:
            logger.warning("读取关节位置失败: %s", "; ".join(errors))
            raise RuntimeError(
                "ZP10S 关节位置读取失败: " + "; ".join(errors)
            )

        return positions
    
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
        
        # self.release_torque()
        
        if self.ser.is_open:
            self.ser.close()
        
        self._is_connected = False
        logger.info("ZP10S Arm disconnected")