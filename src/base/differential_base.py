# src/base/differential_base.py
"""差分底盘实现模块 - PID闭环控制"""

import time
import math
import threading
import logging
from typing import Dict

from src.abstract.base_interface import BaseInterface
from .drivers import MotorDriverProtocol

logger = logging.getLogger(__name__)


class PIDController:
    """PID控制器 - 支持输出限幅和变化率限制"""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_limit: float = 80.0, max_rate: float = 5.0):
        """
        :param kp: 比例系数
        :param ki: 积分系数
        :param kd: 微分系数
        :param output_limit: 输出绝对值上限（PWM百分比），防止过载
        :param max_rate: 每周期最大输出变化量（PWM百分比），防止PWM突变
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.max_rate = max_rate
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_output = 0.0

    def reset(self):
        """重置PID状态"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_output = 0.0

    def compute(self, setpoint: float, measured: float, dt: float) -> float:
        """
        :param setpoint: 目标值（RPM）
        :param measured: 测量值（RPM）
        :param dt: 时间间隔（秒）
        :return: PID输出（PWM百分比）
        """
        error = setpoint - measured

        # 积分项（带抗饱和限幅）
        self.integral += error * dt
        self.integral = max(-self.output_limit, min(self.output_limit, self.integral))

        # 微分项
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # PID输出
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # 输出限幅
        output = max(-self.output_limit, min(self.output_limit, output))

        # 变化率限制（防止PWM突变，保护电机和驱动器）
        output = max(self.last_output - self.max_rate,
                     min(self.last_output + self.max_rate, output))

        self.last_output = output
        return output


class DifferentialBase(BaseInterface):
    """差分底盘实现 - 传统两轮差速驱动（PID闭环控制）

    使用ESP32-C3驱动板的get_rpm()获取实际轮速反馈，
    在base层进行PID计算直接输出PWM，替换驱动板原有的单级PID。
    """

    def __init__(self, config: Dict):
        """
        :param config: 配置字典
            - driver: 电机驱动实例（需支持get_rpm()）
            - wheel_radius: 轮半径（m）
            - wheel_base: 轮距（m）
            - max_linear_speed: 最大线速度（m/s）
            - max_angular_speed: 最大角速度（rad/s）
            - pid: PID配置对象（可选，含kp/ki/kd/output_limit/max_rate）
            - direction_forward: 电机方向修正系数（1或-1），对应驱动的direction_forward
        """
        self.driver: MotorDriverProtocol = config.get('driver')
        self.wheel_radius = config.get('wheel_radius', 0.042)
        self.wheel_base = config.get('wheel_base', 0.195)
        self.max_linear_speed = config.get('max_linear_speed', 0.4)
        self.max_angular_speed = config.get('max_angular_speed', 1.0)
        self.direction_forward = config.get('direction_forward', 1)

        # PID参数：优先从config['pid']读取，其次从config直接读取，最后用默认值
        pid_config = config.get('pid')
        if pid_config is not None:
            kp = getattr(pid_config, 'kp', config.get('kp', 0.5))
            ki = getattr(pid_config, 'ki', config.get('ki', 0.1))
            kd = getattr(pid_config, 'kd', config.get('kd', 0.05))
            output_limit = getattr(pid_config, 'output_limit', config.get('output_limit', 80))
            max_rate = getattr(pid_config, 'max_rate', config.get('max_rate', 5))
        else:
            kp = config.get('kp', 0.5)
            ki = config.get('ki', 0.1)
            kd = config.get('kd', 0.05)
            output_limit = config.get('output_limit', 80)
            max_rate = config.get('max_rate', 5)

        self.left_pid = PIDController(kp, ki, kd, output_limit, max_rate)
        self.right_pid = PIDController(kp, ki, kd, output_limit, max_rate)

        self.last_time = time.perf_counter()
        self.vx = 0.0  # 前后速度 (m/s)
        self.vy = 0.0  # 左右速度（差分底盘为0）
        self.vw = 0.0  # 旋转速度 (rad/s)
        self.LOOP_TIME = 0.02  # 控制周期20ms（50Hz），兼顾UART通信延迟
        self.running = True
        self.thread = threading.Thread(target=self._control_loop)
        self.thread.daemon = True
        self.thread.start()
        logger.info(f"Differential Base initialized (PID closed-loop, "
                    f"wheel_radius={self.wheel_radius}m, wheel_base={self.wheel_base}m, "
                    f"kp={kp}, ki={ki}, kd={kd}, output_limit={output_limit}, max_rate={max_rate})")

    def _control_loop(self):
        """PID闭环控制循环

        坐标系转换（确保PID工作在机器人坐标系，与vx/vw方向一致）：
        1. 运动学逆解：vx, vw(机器人坐标系) → 左右轮目标RPM(机器人坐标系)
        2. get_rpm()获取实际RPM(电机坐标系)，乘direction_forward转为机器人坐标系
        3. PID计算(机器人坐标系)
        4. PID输出乘direction_forward还原为电机坐标系，传给set_speeds()
        """
        while self.running:
            now = time.perf_counter()
            dt = max(0.001, now - self.last_time)
            self.last_time = now

            # 运动学逆解：vx, vw → 左右轮目标RPM（机器人坐标系）
            vx_rpm = (self.vx * 60) / (2 * math.pi * self.wheel_radius)
            wheel_speed_diff = self.vw * self.wheel_base / 2
            vw_rpm = (wheel_speed_diff * 60) / (2 * math.pi * self.wheel_radius)

            target_left = vx_rpm - vw_rpm
            target_right = vx_rpm + vw_rpm

            # 获取实际RPM反馈（电机坐标系）
            actual_left_raw, actual_right_raw = self.driver.get_rpm()

            # 转换到机器人坐标系：乘direction_forward
            actual_left = actual_left_raw * self.direction_forward
            actual_right = actual_right_raw * self.direction_forward

            # PID计算PWM输出（机器人坐标系）
            left_pwm = self.left_pid.compute(target_left, actual_left, dt)
            right_pwm = self.right_pid.compute(target_right, actual_right, dt)

            # 还原到电机坐标系：乘direction_forward，再传给set_speeds()
            left_pwm_motor = left_pwm * self.direction_forward
            right_pwm_motor = right_pwm * self.direction_forward

            # 发送PWM指令
            self.driver.set_speeds(
                int(round(left_pwm_motor)), int(round(right_pwm_motor)))

            logger.debug(f"target=({target_left:.1f},{target_right:.1f})rpm, "
                         f"actual=({actual_left:.1f},{actual_right:.1f})rpm, "
                         f"pwm=({left_pwm_motor:.1f},{right_pwm_motor:.1f})")

            time.sleep(self.LOOP_TIME)

    def move(self, x: float, y: float, w: float) -> None:
        """控制底盘运动

        Args:
            x: 前后方向速度 (m/s)，正值前进，负值后退
            y: 左右方向速度 (m/s) - 差分底盘忽略此参数
            w: 旋转角速度 (rad/s)，正值顺时针，负值逆时针
        """
        self.vx = max(-self.max_linear_speed, min(self.max_linear_speed, x))
        self.vw = max(-self.max_angular_speed, min(self.max_angular_speed, w))

    def stop(self) -> None:
        """停止运动"""
        self.vx = 0.0
        self.vy = 0.0
        self.vw = 0.0
        self.left_pid.reset()
        self.right_pid.reset()
        self.driver.stop()

    def cleanup(self) -> None:
        """释放资源"""
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1)
        self.driver.cleanup()
        logger.info("Differential Base cleaned up")
