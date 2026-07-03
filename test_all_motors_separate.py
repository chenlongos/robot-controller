#!/usr/bin/env python3
"""测试四个电机分别转动（从配置文件读取配置）"""
import os
import time

if not os.path.exists("/dev/gpiochip1"):
    print("错误：GPIO 硬件不可用")
    exit(1)

print("=== 测试四个电机分别转动 ===")
print()

from src.base.motor import Motor
from src.config_loader import load_config

# 从配置文件读取配置
config = load_config()
base_config = config.device.hardware.base

# 测试速度列表（5种不同速度）
TEST_SPEEDS = [10, 30, 60]  # RPM

motors = []

try:
    # 初始化所有电机（从配置文件读取）
    print("初始化四个电机...")
    motor_list = []
    for motor_cfg in base_config.motors:
        motor = Motor(
            name=motor_cfg.name,
            gpio_in2=motor_cfg.in2,
            gpio_in1=motor_cfg.in1,
            gpio_phase_a=motor_cfg.phase_A,
            gpio_phase_b=motor_cfg.phase_B,
            pwm_chip=motor_cfg.pwm_chip,
            pwm_channel=motor_cfg.pwm_channel,
            kp=base_config.pid.kp,
            ki=base_config.pid.ki,
            kd=base_config.pid.kd,
            direction_inverted=motor_cfg.direction_inverted
        )
        motor_list.append(motor)
        print(f"  ✓ {motor_cfg.name} 初始化成功")
    print()
    
    # 测试每个电机的正反转和不同速度
    for motor in motor_list:
        print(f"===== 测试 {motor.name} 电机 =====")
        print()
        
        # 测试正转（5种速度）
        print("1. 测试正转（5种速度）")
        for speed in TEST_SPEEDS:
            print(f"   正转速度: {speed} RPM")
            motor.set_speed(speed)
            start_time = time.time()
            while time.time() - start_time < 1.0:
                if motor.get_speed() == speed:
                    break
                motor.update(0.01)
                time.sleep(0.01)
            motor.set_speed(0)
            while motor.get_speed() != 0:
                motor.update(0.01)
                time.sleep(0.01)
            print(f"   ✓ 完成")
            print()
            input("按任意键继续测试...")
            print()
        
        # 测试反转（5种速度）
        print("2. 测试反转（5种速度）")
        for speed in TEST_SPEEDS:
            print(f"   反转速度: -{speed} RPM")
            motor.set_speed(-speed)
            start_time = time.time()
            while time.time() - start_time < 1.0:
                if motor.get_speed() == -speed:
                    break
                motor.update(0.01)
                time.sleep(0.01)
            motor.set_speed(0)
            while motor.get_speed() != 0:
                motor.update(0.01)
                time.sleep(0.01)
            print(f"   ✓ 完成")
            print()
            input("按任意键继续测试...")
            print()
        
        print()
        print(f"===== {motor.name} 电机测试完成 =====")
        print()
        input("按任意键继续下一个电机...")
        print()
    
    print("=== 所有电机测试完成 ===")
    
finally:
    # 清理所有电机
    print("清理电机资源...")
    for motor in motor_list:
        motor.cleanup()
    print("✓ 清理完成")