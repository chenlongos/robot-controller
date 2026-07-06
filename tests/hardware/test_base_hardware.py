"""底盘控制器硬件测试 - 需要真实硬件"""
import pytest
import time
import os
from src.base.d24a_jgb37 import Motor
from src.base.differential_base import DifferentialBase
from src.base.mecanum_base import MecanumBase
from src.config_loader import load_config

class TestBaseControllerHardware:
    """底盘控制器硬件测试类"""
    
    def test_differential_controller_movement(self):
        """测试差分底盘控制器运动功能"""
        # 检查硬件是否可用
        if not os.path.exists("/dev/gpiochip1"):
            pytest.skip("GPIO 硬件不可用，跳过硬件测试")
        
        config = load_config()
        controller = None
        
        try:
            # 获取底盘配置
            base_config = config.device.hardware.base
            
            # 创建电机配置字典
            motors_config = {}
            for motor_cfg in base_config.motors:
                motors_config[motor_cfg.name] = {
                    'name': motor_cfg.name,
                    'gpio_in2': motor_cfg.in2,
                    'gpio_in1': motor_cfg.in1,
                    'gpio_phase_a': motor_cfg.phase_A,
                    'gpio_phase_b': motor_cfg.phase_B,
                    'pwm_chip': motor_cfg.pwm_chip,
                    'pwm_channel': motor_cfg.pwm_channel,
                    'kp': base_config.pid.kp,
                    'ki': base_config.pid.ki,
                    'kd': base_config.pid.kd,
                    'direction_inverted': motor_cfg.direction_inverted
                }
            
            # 创建底盘控制器配置
            controller_config = {
                'motors': motors_config,
                'max_speed': base_config.max_linear_speed * 1000  # 转换为 RPM
            }
            
            # 使用差分底盘控制器
            controller = DifferentialBase(controller_config)
            
            print("\n测试差分底盘控制器...")
            
            # 前进测试
            print("  前进 1 秒...")
            controller.move(50, 0, 0)  # 使用 move 方法
            time.sleep(1)
            
            # 停止
            controller.stop()
            time.sleep(0.5)
            
            # 后退测试
            print("  后退 1 秒...")
            controller.move(-50, 0, 0)  # 使用 move 方法
            time.sleep(1)
            
            # 停止
            controller.stop()
            time.sleep(0.5)
            
            # 左转测试
            print("  左转 1 秒...")
            controller.move(0, 0, 30)  # 使用 move 方法
            time.sleep(1)
            
            # 停止
            controller.stop()
            time.sleep(0.5)
            
            # 右转测试
            print("  右转 1 秒...")
            controller.move(0, 0, -30)  # 使用 move 方法
            time.sleep(1)
            
            # 停止
            controller.stop()
            time.sleep(0.5)
            
            print("  ✓ 差分底盘控制器测试完成")
            
        finally:
            if controller:
                controller.cleanup()
    
    def test_differential_controller_simple_move(self):
        """测试差分底盘控制器简单移动（用于快速验证）"""
        # 检查硬件是否可用
        if not os.path.exists("/dev/gpiochip1"):
            pytest.skip("GPIO 硬件不可用，跳过硬件测试")
        
        config = load_config()
        controller = None
        
        try:
            # 获取底盘配置
            base_config = config.device.hardware.base
            
            # 创建电机配置字典
            motors_config = {}
            for motor_cfg in base_config.motors:
                motors_config[motor_cfg.name] = {
                    'name': motor_cfg.name,
                    'gpio_in2': motor_cfg.in2,
                    'gpio_in1': motor_cfg.in1,
                    'gpio_phase_a': motor_cfg.phase_A,
                    'gpio_phase_b': motor_cfg.phase_B,
                    'pwm_chip': motor_cfg.pwm_chip,
                    'pwm_channel': motor_cfg.pwm_channel,
                    'kp': base_config.pid.kp,
                    'ki': base_config.pid.ki,
                    'kd': base_config.pid.kd,
                    'direction_inverted': motor_cfg.direction_inverted
                }
            
            # 创建底盘控制器配置
            controller_config = {
                'motors': motors_config,
                'max_speed': base_config.max_linear_speed * 1000  # 转换为 RPM
            }
            
            # 使用差分底盘控制器
            controller = DifferentialBase(controller_config)
            
            print("\n测试差分底盘控制器简单移动...")
            
            # 前进测试
            print("  前进 2 秒...")
            controller.move(60, 0, 0)  # 使用 move 方法
            time.sleep(2)
            
            # 停止
            controller.stop()
            time.sleep(1)
            
            print("  ✓ 简单移动测试完成")
            
        finally:
            if controller:
                controller.cleanup()