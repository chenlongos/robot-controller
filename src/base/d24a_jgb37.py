# src/base/d24a_jgb37.py
"""D24A-JGB37电机控制模块 - 底盘的核心组件"""

from periphery import GPIO, PWM
import time
import threading, ctypes
import logging
from typing import Dict, List

logger = logging.getLogger(__name__)

def get_gpio_chip_and_line(global_gpio_num):
    """
    将全局 GPIO 编号转换为 (Chip Path, Line Offset)
    规则: 每 32 个 GPIO 为一个 chip
    """
    chip_index = global_gpio_num // 32
    line_offset = global_gpio_num % 32
    chip_path = f"/dev/gpiochip{chip_index}"
    return chip_path, line_offset

def get_tid():
    return ctypes.CDLL('libc.so.6').syscall(186)

class EncoderCounter:
    """编码器计数器类"""
    def __init__(self, global_pin_a, global_pin_b):
        self.count = 0
        self.last_state = 0
        self.lock = threading.Lock()
        self.running = True
        self.gpio_a = None
        self.gpio_b = None
        self.table = [
            [  0, -1, +1,  0 ],  # last 00
            [ +1,  0,  0, -1 ],  # last 01
            [ -1,  0,  0, +1 ],  # last 10
            [  0, +1, -1,  0 ]   # last 11
        ]
        
        chip_path_a, line_a = get_gpio_chip_and_line(global_pin_a)
        chip_path_b, line_b = get_gpio_chip_and_line(global_pin_b)
        
        try:
            self.gpio_a = GPIO(chip_path_a, line_a, "in", edge="both", bias="pull_up")
            self.gpio_b = GPIO(chip_path_b, line_b, "in", edge="both", bias="pull_up")

            self.curr_a = 1 if self.gpio_a.read() else 0
            self.curr_b = 1 if self.gpio_b.read() else 0
            self.last_state = self.curr_a << 1 | self.curr_b
            
            self.thread = threading.Thread(target=self._poll_events)
            self.thread.daemon = True
            self.thread.start()
        except Exception as e:
            raise RuntimeError(f"Failed to init Encoder (A:{global_pin_a}, B:{global_pin_b}): {e}")

    def _poll_events(self):
        logger.info(f"Encoder thread started for GPIO A:{self.gpio_a.line}, B:{self.gpio_b.line}")
        gpios = [self.gpio_a, self.gpio_b]
        while self.running:
            triggered_gpios = GPIO.poll_multiple(gpios, 0.001)
            if not triggered_gpios:
                continue

            for gpio in triggered_gpios:
                try:
                    event = gpio.read_event()
                    if gpio == self.gpio_a:
                        if event.edge == "falling":
                            self.curr_a = 0
                        else:
                            self.curr_a = 1
                    else:
                        if event.edge == "falling":
                            self.curr_b = 0
                        else:
                            self.curr_b = 1
                except Exception as e:
                    logger.error(f"read event : {e}")
                    continue

            current_state = (self.curr_a << 1) | self.curr_b
            if current_state != self.last_state:
                delta = self.table[self.last_state][current_state]
                with self.lock:
                    self.count += delta
                self.last_state = current_state

    def get_speed(self, interval):
        """获取编码器速度（计数/秒）
        
        :param interval: 时间间隔（秒）
        :return: 编码器计数/秒（counts/s），即每秒编码器脉冲数
        """
        with self.lock:
            val = self.count
            self.count = 0
        return val / interval

    def cleanup(self):
        self.running = False
        if self.gpio_a: self.gpio_a.close()
        if self.gpio_b: self.gpio_b.close()

class PIDController:
    """PID控制器类"""
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp, self.ki, self.kd, self.setpoint = kp, ki, kd, setpoint
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.alpha = 0.01  # 微分滤波器系数
        self.filtered_derivative = 0.0

    def compute(self, current_value):
        now = time.time()
        dt = max(0.001, now - self.last_time)
        error = self.setpoint - current_value

        P = self.kp * error
        self.integral = max(-100, min(100, self.integral + error * dt))
        I = self.ki * self.integral
        derivative = (error - self.prev_error) / dt
        self.filtered_derivative = self.alpha * derivative + (1 - self.alpha) * self.filtered_derivative
        D = self.kd * self.filtered_derivative

        self.prev_error = error
        self.last_time = now
        return P + I + D

class Motor:
    """电机类 - 底盘的本质组件"""
    def __init__(self, name, gpio_in2, gpio_in1, gpio_phase_a, gpio_phase_b, 
                 pwm_chip, pwm_channel, kp, ki, kd, direction_inverted=False):
        """
        :param name: 电机名称 (FL, FR, BL, BR)
        :param gpio_in2: IN2引脚编号
        :param gpio_in1: IN1引脚编号
        :param gpio_phase_a: 编码器A相引脚
        :param gpio_phase_b: 编码器B相引脚
        :param pwm_chip: PWM芯片编号
        :param pwm_channel: PWM通道编号
        :param kp, ki, kd: PID参数
        :param direction_inverted: 是否反转方向
        """
        self.name = name
        # self.CPR = 880  # 530RPM电机：20减速比 * 11线数 * 4(四倍频)
        self.CPR = 1320  # 330RPM电机：30减速比 * 11线数 * 4(四倍频)
        self.DEADZONE_MIN = 5.0  # 死区 (%)
        self.PWM_FREQ = 20000    # Hz
        self.pid = PIDController(kp, ki, kd, 0)
        self.in1_gpio = None
        self.in2_gpio = None
        self.pwm = None
        self.direction = -1 if direction_inverted else 1
        
        # 1. 初始化方向引脚 (IN1, IN2)
        try:
            chip_in2, line_in2 = get_gpio_chip_and_line(gpio_in2)
            chip_in1, line_in1 = get_gpio_chip_and_line(gpio_in1)
            
            self.in2_gpio = GPIO(chip_in2, line_in2, "out")
            self.in1_gpio = GPIO(chip_in1, line_in1, "out")
            
            self.in2_gpio.write(False)
            self.in1_gpio.write(False)
            
            logger.info(f"  -> DIR Initialized: IN2(Global:{gpio_in2}->{chip_in2}:{line_in2}), IN1(Global:{gpio_in1}->{chip_in1}:{line_in1})")
            
        except Exception as e:
            raise RuntimeError(f"Failed to init DIR GPIOs for {name}: {e}")

        # 2. 初始化编码器
        try:
            self.encoder = EncoderCounter(gpio_phase_a, gpio_phase_b)
            chip_a, line_a = get_gpio_chip_and_line(gpio_phase_a)
            logger.info(f"  -> Encoder Initialized: A(Global:{gpio_phase_a}->{chip_a}:{line_a})")
        except Exception as e:
            raise RuntimeError(f"Failed to init Encoder for {name}: {e}")

        # 3. 初始化 PWM
        try:
            self.pwm = PWM(pwm_chip, pwm_channel)
            self.pwm.frequency = self.PWM_FREQ
            self.pwm.duty_cycle = 0
            self.pwm.polarity = "normal"
            self.pwm.enable()
            logger.info(f"  -> PWM Initialized: Chip{pwm_chip}, Ch{pwm_channel}")
            
        except Exception as e:
            raise RuntimeError(f"Failed to init PWM for {name}: {e}")

    def set_speed(self, target_speed):
        """设置电机目标速度（RPM）
        
        :param target_speed: 目标速度（转/分钟，RPM），正值为正转，负值为反转
        """
        self.pid.setpoint = target_speed

    def get_speed(self, dt=0.001):
        """获取电机当前速度（RPM）
        
        :param dt: 采样时间间隔（秒），用于计算编码器速度
        :return: 电机当前速度（转/分钟，RPM），正值为正转，负值为反转
        
        速度转换公式：
        编码器速度（counts/s）→ 电机转速（RPM）
        RPM = (encoder_counts / dt) * 60 / CPR
        其中 CPR = 编码器每转脉冲数（含四倍频）
        """
        # 先获取编码器原始速度（计数/秒）
        encoder_speed_counts_per_sec = self.encoder.get_speed(dt)
        # 转换为 RPM（转/分钟）
        speed_rpm = self.direction * encoder_speed_counts_per_sec * 60 / self.CPR
        return speed_rpm

    def update(self, dt):
        speed = self.get_speed(dt)

        # 避免进入死区效应
        if self.pid.setpoint == 0 and abs(speed) < self.DEADZONE_MIN:
            self._drive(0, 1)
            return speed, 0
        
        output = self.pid.compute(speed)
        duty = abs(output)
        direction = (1 if output >= 0 else -1) * self.direction
        
        duty = max(0, min(100, duty))
        if 0 < duty < self.DEADZONE_MIN:
            duty = self.DEADZONE_MIN

        logger.debug(f"{self.name}, target={self.pid.setpoint:.1f}RPM, actual={speed:.1f}RPM, output={output:.1f}, duty={duty:.1f}%, direction={direction}")
        self._drive(duty, direction)
        return speed, duty

    def _drive(self, duty_percent, direction):
        if direction == 1:
            self.in1_gpio.write(True)
            self.in2_gpio.write(False)
        else:
            self.in1_gpio.write(False)
            self.in2_gpio.write(True)
            
        if self.pwm:
            self.pwm.duty_cycle = duty_percent / 100.0

    def cleanup(self):
        if self.in1_gpio: self.in1_gpio.close()
        if self.in2_gpio: self.in2_gpio.close()
        if self.encoder: self.encoder.cleanup()
        if self.pwm:
            self.pwm.duty_cycle = 0
            self.pwm.disable()
            self.pwm.close()
