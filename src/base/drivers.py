# src/base/drivers.py
"""电机驱动接口和实现"""

from typing import Protocol, runtime_checkable, Optional
import struct
import time
import logging

try:
    import serial
except ImportError:
    serial = None

logger = logging.getLogger(__name__)

FRAME_H1 = 0xAA
FRAME_H2 = 0x55

CMD_INIT = 0x01
CMD_CONFIG = 0x02
CMD_SET_SPEED = 0x10
CMD_SET_SPEEDS = 0x13
CMD_STOP = 0x11
CMD_BRAKE = 0x12
CMD_GET_RPM = 0x20
CMD_GET_STATUS = 0x21
CMD_RESET = 0xFF

RSP_ACK = 0x80
RSP_NACK = 0x81
RSP_RPM_DATA = 0x90
RSP_STATUS = 0x91


@runtime_checkable
class MotorDriverProtocol(Protocol):
    """电机驱动协议接口"""
    
    def set_speeds(self, *args) -> None:
        """设置轮速度（-100~100，百分比）
        差速底盘: set_speeds(left, right)
        麦轮底盘: set_speeds(fl, fr, bl, br)
        """
        ...
    
    def stop(self) -> None:
        """停止电机"""
        ...
    
    def cleanup(self) -> None:
        """释放资源"""
        ...


class D24aJgb37Driver:
    """D24A驱动板 + JGB37-520电机驱动实现"""
    
    def __init__(self, motors_config, pid_config):
        from .d24a_jgb37 import Motor
        
        self.motors = {}
        for motor_data in motors_config:
            self.motors[motor_data.name] = Motor(
                name=motor_data.name,
                gpio_in2=motor_data.in2,
                gpio_in1=motor_data.in1,
                gpio_phase_a=motor_data.phase_A,
                gpio_phase_b=motor_data.phase_B,
                pwm_chip=motor_data.pwm_chip,
                pwm_channel=motor_data.pwm_channel,
                kp=pid_config.kp,
                ki=pid_config.ki,
                kd=pid_config.kd,
                direction_inverted=motor_data.direction_inverted
            )
        
        self.max_speed = 400
        logger.info("D24A-JGB37 Driver initialized")
    
    def set_speeds(self, *args) -> None:
        """设置轮速度（RPM）
        差速底盘: set_speeds(left, right)
        麦轮底盘: set_speeds(fl, fr, bl, br)
        """
        if len(args) == 2:
            left, right = args
            for name, motor in self.motors.items():
                if 'left' in name.lower():
                    motor.set_speed(left)
                elif 'right' in name.lower():
                    motor.set_speed(right)
        elif len(args) == 4:
            fl, fr, bl, br = args
            for name, motor in self.motors.items():
                if name == 'front_left':
                    motor.set_speed(fl)
                elif name == 'front_right':
                    motor.set_speed(fr)
                elif name == 'back_left':
                    motor.set_speed(bl)
                elif name == 'back_right':
                    motor.set_speed(br)
    
    def update(self, dt):
        """更新电机PID控制（调用此方法让PID生效）"""
        for motor in self.motors.values():
            motor.update(dt)
    
    def stop(self) -> None:
        """停止电机"""
        for motor in self.motors.values():
            motor.set_speed(0)
    
    def cleanup(self) -> None:
        """释放资源"""
        for motor in self.motors.values():
            motor.cleanup()
        logger.info("D24A-JGB37 Driver cleaned up")


class Esp32C3TtDriver:
    """ESP32-C3 TT马达UART驱动实现"""
    
    def __init__(self, uart_config):
        self.port = uart_config.port
        self.baudrate = uart_config.baudrate
        self.ppr = uart_config.ppr
        self.pwm_freq = uart_config.pwm_freq
        
        self.ser = None
        self._init_serial()
        
        if not self._send_cmd(CMD_INIT):
            raise RuntimeError("ESP32-C3 init failed")
        if not self._config(self.ppr, self.pwm_freq):
            raise RuntimeError("ESP32-C3 config failed")
        
        logger.info(f"ESP32-C3 TT Driver initialized (port={self.port})")
    
    def _init_serial(self):
        if serial is None:
            raise ImportError("pyserial is not installed")
        
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
        time.sleep(0.5)
        self.ser.reset_input_buffer()
    
    def _build_frame(self, cmd: int, payload: bytes = b"") -> bytes:
        chk = cmd ^ len(payload)
        for b in payload:
            chk ^= b
        return bytes([FRAME_H1, FRAME_H2, cmd, len(payload)]) + payload + bytes([chk])
    
    def _recv_frame(self, timeout: float = 0.2) -> Optional[dict]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                if self.ser.in_waiting < 4:
                    time.sleep(0.01)
                    continue
                h1 = self.ser.read(1)
                if not h1 or h1[0] != FRAME_H1:
                    continue
                h2 = self.ser.read(1)
                if not h2 or h2[0] != FRAME_H2:
                    continue
                header = self.ser.read(2)
                if len(header) < 2:
                    continue
                cmd, length = header[0], header[1]
                payload = self.ser.read(length) if length else b""
                chk_b = self.ser.read(1)
                if not chk_b:
                    continue
                chk = cmd ^ length
                for b in payload:
                    chk ^= b
                if chk != chk_b[0]:
                    return None
                return {"cmd": cmd, "payload": bytes(payload)}
            except serial.SerialException:
                return None
        return None
    
    def _send_cmd(self, cmd: int, payload: bytes = b"", timeout: float = 0.2) -> bool:
        self.ser.reset_input_buffer()
        self.ser.write(self._build_frame(cmd, payload))
        self.ser.flush()
        rsp = self._recv_frame(timeout)
        return rsp is not None and rsp["cmd"] == RSP_ACK
    
    def _send_cmd_noresp(self, cmd: int, payload: bytes = b"") -> None:
        self.ser.write(self._build_frame(cmd, payload))
        self.ser.flush()
    
    def _config(self, ppr: int, pwm_freq: int) -> bool:
        payload = struct.pack(">HH", ppr, pwm_freq)
        return self._send_cmd(CMD_CONFIG, payload)
    
    def set_speeds(self, left: int, right: int) -> None:
        """设置左右轮速度（-100~100，百分比）"""
        left_pwm = max(-100, min(100, left))
        right_pwm = max(-100, min(100, right))
        
        payload = struct.pack(">hh", left_pwm, right_pwm)
        self._send_cmd_noresp(CMD_SET_SPEEDS, payload)
    
    def update(self, dt):
        """ESP32-C3驱动无需PID更新（PID在ESP32端运行）"""
        pass
    
    def stop(self) -> None:
        """停止电机"""
        self._send_cmd_noresp(CMD_STOP, bytes([2]))
    
    def cleanup(self) -> None:
        """释放资源"""
        self.stop()
        if self.ser and self.ser.is_open:
            self.ser.close()
        logger.info("ESP32-C3 TT Driver cleaned up")