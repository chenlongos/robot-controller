#!/usr/bin/env python3
"""差速轮底盘测试文件"""
import os
import sys
import time

# 添加项目路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.base.differential_base import DifferentialBase
from src.config_loader import load_config

def test_differential_base_movement():
    """测试差速轮底盘的各种运动功能"""
    # 检查硬件是否可用
    if not os.path.exists("/dev/gpiochip1"):
        print("错误：GPIO 硬件不可用")
        return
    
    print("=== 差速轮底盘功能测试 ===")
    print()
    
    config = load_config()
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
        'max_speed': base_config.max_linear_speed * 1000,  # 转换为 RPM
        'wheel_radius': base_config.wheel_radius,          # 车轮半径（米）
        'wheel_base': base_config.wheel_base              # 轮距（米）
    }
    
    controller = None
    
    try:
        print("初始化差速轮底盘控制器...")
        controller = DifferentialBase(controller_config)
        print("✓ 控制器初始化成功")
        print(f"电机数量: {len(controller.motors)}")
        print()
        
        # 测试前进
        print("1. 测试前进")
        print("   设置前进速度: x=300 mm/s")
        controller.move(x=30, y=0, w=0)
        time.sleep(2)
        controller.stop()
        print("   ✓ 前进测试完成")
        print()
        input("按任意键继续测试...")
        
        # 测试后退
        print("2. 测试后退")
        print("   设置后退速度: x=-300 mm/s")
        controller.move(x=-300, y=0, w=0)
        time.sleep(2)
        controller.stop()
        print("   ✓ 后退测试完成")
        print()
        input("按任意键继续测试...")
        
        # 测试原地左转
        print("3. 测试原地左转")
        print("   设置左转角速度: w=90 度/s")
        controller.move(x=0, y=0, w=90)
        time.sleep(2)
        controller.stop()
        print("   ✓ 原地左转测试完成")
        print()
        input("按任意键继续测试...")
        
        # 测试原地右转
        print("4. 测试原地右转")
        print("   设置右转角速度: w=-90 度/s")
        controller.move(x=0, y=0, w=-90)
        time.sleep(2)
        controller.stop()
        print("   ✓ 原地右转测试完成")
        print()
        input("按任意键继续测试...")
        
        # 测试前进加左转（弧线运动）
        print("5. 测试前进加左转（弧线运动）")
        print("   设置: x=200 mm/s, w=45 度/s")
        controller.move(x=200, y=0, w=45)
        time.sleep(2)
        controller.stop()
        print("   ✓ 弧线运动测试完成")
        print()
        input("按任意键继续测试...")
        
        # 测试前进加右转
        print("6. 测试前进加右转")
        print("   设置: x=200 mm/s, w=-45 度/s")
        controller.move(x=200, y=0, w=-45)
        time.sleep(2)
        controller.stop()
        print("   ✓ 前进右转测试完成")
        print()
        
        print("=== 所有测试完成 ===")
        
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        if controller:
            print("\n清理控制器资源...")
            controller.cleanup()
            print("✓ 资源清理完成")

if __name__ == "__main__":
    test_differential_base_movement()