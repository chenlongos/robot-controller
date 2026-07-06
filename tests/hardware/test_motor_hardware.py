"""电机硬件测试 - 测试 src/base/d24a_jgb37.py 中的 Motor 类实现"""
import pytest
import time
import os
from src.base.d24a_jgb37 import Motor
from src.config_loader import load_config

# 从配置文件加载电机配置
_config = load_config()
MOTOR_CONFIGS_FROM_CONFIG = [
    {
        "name": cfg.name.replace("front_", "F").replace("back_", "B").replace("left", "L").replace("right", "R"),
        "in2": cfg.in2,
        "in1": cfg.in1,
        "phase_a": cfg.phase_A,
        "phase_b": cfg.phase_B,
        "pwm_chip": cfg.pwm_chip,
        "pwm_channel": cfg.pwm_channel,
        "direction_inverted": cfg.direction_inverted,
        "expected_direction": -1 if cfg.direction_inverted else 1
    }
    for cfg in _config.device.hardware.base.motors
]

class TestMotorHardware:
    """电机硬件测试类 - 测试 Motor 类的实际硬件控制"""
    
    def test_motor_class_all_motors_spin(self):
        """测试 Motor 类控制所有电机依次转动"""
        # 检查硬件是否可用
        gpiochips = [f"/dev/gpiochip{i}" for i in range(10) if os.path.exists(f"/dev/gpiochip{i}")]
        if not gpiochips:
            pytest.skip("GPIO 硬件不可用，跳过硬件测试")
        
        motors = []
        
        try:
            # 使用 Motor 类初始化所有电机
            print("\n使用 Motor 类初始化所有电机...")
            for config in MOTOR_CONFIGS_FROM_CONFIG:
                motor = Motor(
                    name=config["name"],
                    gpio_in2=config["in2"],
                    gpio_in1=config["in1"],
                    gpio_phase_a=config["phase_a"],
                    gpio_phase_b=config["phase_b"],
                    pwm_chip=config["pwm_chip"],
                    pwm_channel=config["pwm_channel"],
                    kp=_config.device.hardware.base.pid.kp,
                    ki=_config.device.hardware.base.pid.ki,
                    kd=_config.device.hardware.base.pid.kd,
                    direction_inverted=config["direction_inverted"]
                )
                motors.append(motor)
                print(f"  ✓ 初始化电机 {config['name']}")
            
            # 测试 set_speed 和 update 方法
            print("\n测试 Motor.set_speed() 和 Motor.update()...")
            for motor in motors:
                print(f"\n启动电机 {motor.name}...")
                
                # 设置目标速度
                motor.set_speed(60)  # 60 RPM
                
                # 运行控制循环1秒
                start_time = time.time()
                while time.time() - start_time < 1.0:
                    speed, duty = motor.update(dt=0.01)
                    time.sleep(0.01)
                
                # 停止电机
                motor.set_speed(0)
                motor.update(dt=0.01)
                print(f"  ✓ 电机 {motor.name} 已停止")
                time.sleep(0.5)
            
            print("\n✓ Motor 类硬件测试完成！")
            
        except Exception as e:
            print(f"\n✗ 错误: {e}")
            raise
        
        finally:
            # 清理所有电机
            print("\n清理电机资源...")
            for motor in motors:
                motor.cleanup()
            print("✓ 清理完成")
    
    def test_motor_class_get_speed(self):
        """测试 Motor.get_speed() 方法"""
        # 检查硬件是否可用
        if not os.path.exists("/dev/gpiochip1"):
            pytest.skip("GPIO 硬件不可用，跳过硬件测试")
        
        config = MOTOR_CONFIGS_FROM_CONFIG[0]  # 使用 FL 电机
        motor = None
        
        try:
            # 创建 Motor 实例
            motor = Motor(
                name=config["name"],
                gpio_in2=config["in2"],
                gpio_in1=config["in1"],
                gpio_phase_a=config["phase_a"],
                gpio_phase_b=config["phase_b"],
                pwm_chip=config["pwm_chip"],
                pwm_channel=config["pwm_channel"],
                kp=_config.device.hardware.base.pid.kp,
                ki=_config.device.hardware.base.pid.ki,
                kd=_config.device.hardware.base.pid.kd,
                direction_inverted=config["direction_inverted"]
            )
            
            # 测试 get_speed 方法
            print(f"\n测试 {motor.name} 的 get_speed() 方法...")
            
            # 静止时读取速度
            speed = motor.get_speed()
            print(f"  静止时速度: {speed:.2f} RPM")
            assert isinstance(speed, (int, float)), "get_speed() 应返回数值"
            
            # 让电机转动后读取速度
            motor.set_speed(30)
            for _ in range(50):
                motor.update(dt=0.01)
                time.sleep(0.01)
            
            speed = motor.get_speed()
            print(f"  转动时速度: {speed:.2f} RPM")
            
            # 停止电机
            motor.set_speed(0)
            motor.update(dt=0.01)
            
            print("  ✓ get_speed() 测试完成")
            
        finally:
            if motor:
                motor.cleanup()
    
    def test_motor_class_direction(self):
        """测试 Motor 类的方向控制"""
        # 检查硬件是否可用
        if not os.path.exists("/dev/gpiochip1"):
            pytest.skip("GPIO 硬件不可用，跳过硬件测试")
        
        # 测试正向和反向
        config = MOTOR_CONFIGS_FROM_CONFIG[0]  # 使用 FL 电机
        motor_normal = None
        motor_inverted = None
        
        try:
            # 创建两个电机实例，一个正向，一个反向
            motor_normal = Motor(
                name="FL_NORMAL",
                gpio_in2=config["in2"],
                gpio_in1=config["in1"],
                gpio_phase_a=config["phase_a"],
                gpio_phase_b=config["phase_b"],
                pwm_chip=config["pwm_chip"],
                pwm_channel=config["pwm_channel"],
                kp=_config.device.hardware.base.pid.kp,
                ki=_config.device.hardware.base.pid.ki,
                kd=_config.device.hardware.base.pid.kd,
                direction_inverted=False
            )
            
            print(f"\n测试电机方向控制...")
            print(f"  正向电机 direction: {motor_normal.direction}")
            assert motor_normal.direction == 1, "正向电机 direction 应为 1"
            
            # 测试转动
            motor_normal.set_speed(30)
            for _ in range(30):
                motor_normal.update(dt=0.01)
                time.sleep(0.01)
            motor_normal.set_speed(0)
            motor_normal.update(dt=0.01)
            
            print("  ✓ 方向控制测试完成")
            
        finally:
            if motor_normal:
                motor_normal.cleanup()
            if motor_inverted:
                motor_inverted.cleanup()