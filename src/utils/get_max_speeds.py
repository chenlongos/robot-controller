import sys
import os
import math
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config_loader import load_config
from src.base.drivers import Esp32C3TtDriver
from tests._robot_select import list_robots, select_robot


def find_max_motor_speed(driver, wheel_radius, wheel_base):
    step = 1
    max_speed_cmd = 10
    stabilization_time = 1.5
    sample_count = 5
    
    print("\n正在测试直线运动速度特性...")
    print("=" * 60)
    
    best_rpm = 0
    best_cmd = 0
    data_points = []
    
    for speed_cmd in range(0, max_speed_cmd + 1, step):
        driver.set_speeds(speed_cmd, speed_cmd)
        time.sleep(stabilization_time)
        
        rpm_sum = 0
        valid_samples = 0
        for i in range(sample_count):
            left_rpm, right_rpm = driver.get_rpm()
            left_rpm = abs(left_rpm)
            right_rpm = abs(right_rpm)
            if left_rpm != 0 or right_rpm != 0:
                rpm_sum += (left_rpm + right_rpm) / 2
                valid_samples += 1
            time.sleep(0.1)
        
        if valid_samples > 0:
            avg_rpm = rpm_sum / valid_samples
        else:
            avg_rpm = 0
        
        data_points.append((speed_cmd, avg_rpm))
        
        print(f"  速度指令: {speed_cmd:4d} -> 实际转速: {avg_rpm:6.2f} RPM")
        
        if avg_rpm > best_rpm:
            best_rpm = avg_rpm
            best_cmd = speed_cmd
        elif avg_rpm < best_rpm * 0.9 and speed_cmd > best_cmd + step * 2:
            print(f"\n  检测到转速下降，停止测试")
            break
    
    driver.stop()
    time.sleep(0.5)
    
    return {
        "max_motor_rpm": best_rpm,
        "max_speed_cmd": best_cmd,
        "data_points": data_points,
    }


def find_max_rotational_speed(driver, wheel_radius, wheel_base):
    step = 1
    max_speed_cmd = 15
    stabilization_time = 1.5
    sample_count = 5
    
    print("\n正在测试旋转运动速度特性...")
    print("=" * 60)
    
    best_rpm = 0
    best_cmd = 0
    data_points = []
    
    for speed_cmd in range(0, max_speed_cmd + 1, step):
        driver.set_speeds(speed_cmd, -speed_cmd)
        time.sleep(stabilization_time)
        
        rpm_sum = 0
        valid_samples = 0
        for i in range(sample_count):
            left_rpm, right_rpm = driver.get_rpm()
            left_rpm = abs(left_rpm)
            right_rpm = abs(right_rpm)
            if left_rpm != 0 or right_rpm != 0:
                rpm_sum += (left_rpm + right_rpm) / 2
                valid_samples += 1
            time.sleep(0.1)
        
        if valid_samples > 0:
            avg_rpm = rpm_sum / valid_samples
        else:
            avg_rpm = 0
        
        data_points.append((speed_cmd, avg_rpm))
        
        if avg_rpm > 0:
            v_edge = avg_rpm * 2 * math.pi * wheel_radius / 60
            omega = 2 * v_edge / wheel_base
        else:
            omega = 0
        
        print(f"  速度指令: {speed_cmd:4d} -> 实际转速: {avg_rpm:6.2f} RPM -> 角速度: {omega:.4f} rad/s")
        
        if avg_rpm > best_rpm:
            best_rpm = avg_rpm
            best_cmd = speed_cmd
        elif avg_rpm < best_rpm * 0.9 and speed_cmd > best_cmd + step * 2:
            print(f"\n  检测到转速下降，停止测试")
            # break
    
    driver.stop()
    time.sleep(0.5)
    
    return {
        "max_motor_rpm": best_rpm,
        "max_speed_cmd": best_cmd,
        "data_points": data_points,
    }


def linear_regression(data_points):
    n = len(data_points)
    if n < 2:
        return None
    
    x_sum = sum(x for x, y in data_points)
    y_sum = sum(y for x, y in data_points)
    xy_sum = sum(x * y for x, y in data_points)
    x2_sum = sum(x * x for x, y in data_points)
    
    denom = n * x2_sum - x_sum * x_sum
    if denom == 0:
        return None
    
    slope = (n * xy_sum - x_sum * y_sum) / denom
    intercept = (y_sum - slope * x_sum) / n
    
    ss_tot = sum((y - y_sum / n) ** 2 for x, y in data_points)
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in data_points)
    
    if ss_tot == 0:
        r_squared = 1.0
    else:
        r_squared = 1 - (ss_res / ss_tot)
    
    return {
        "slope": slope,
        "intercept": intercept,
        "r_squared": r_squared,
    }


def select_motion_type():
    print("\n请选择运动类型:")
    print("  1. 直线运动")
    print("  2. 旋转运动")
    
    while True:
        try:
            choice = input("\n请选择 (1-2，默认1): ").strip()
            if choice == "" or choice == "1":
                return "linear"
            elif choice == "2":
                return "rotational"
            else:
                print("请输入 1 或 2")
        except (EOFError, KeyboardInterrupt):
            print("\n已取消")
            return None


def main():
    robot_name = select_robot()
    if robot_name is None:
        return 0
    
    motion_type = select_motion_type()
    if motion_type is None:
        return 0
    
    config = None
    driver = None
    
    try:
        config = load_config(robot_name)
        
        base_config = config.device.hardware.base
        driver_type = base_config.driver
        wheel_radius = base_config.wheel_radius
        wheel_base = base_config.wheel_base
        
        print("\n" + "=" * 60)
        print(f"机器人: {robot_name}")
        print(f"底盘类型: {base_config.type}")
        print(f"驱动类型: {driver_type}")
        if motion_type == "linear":
            print("运动类型: 直线运动")
        else:
            print("运动类型: 旋转运动")
        print("=" * 60)
        
        if driver_type != "esp32_c3_tt":
            print(f"错误: 当前仅支持 esp32_c3_tt 驱动类型")
            return 1
        
        if base_config.uart is None:
            print("错误: 未找到 UART 配置")
            return 1
        
        uart_port = base_config.uart.port
        print(f"\n串口配置:")
        print(f"  端口: {uart_port}")
        print(f"  波特率: {base_config.uart.baudrate}")
        print(f"  PPR: {base_config.uart.ppr}")
        print(f"  PWM频率: {base_config.uart.pwm_freq}")
        
        print("\n正在初始化驱动...")
        try:
            driver = Esp32C3TtDriver(base_config.uart)
            print("  ✓ 驱动初始化成功")
        except Exception as e:
            print(f"  ✗ 驱动初始化失败: {e}")
            print(f"    请检查串口设备 {uart_port} 是否可用")
            return 1
        
        if motion_type == "linear":
            result = find_max_motor_speed(driver, wheel_radius, wheel_base)
        else:
            result = find_max_rotational_speed(driver, wheel_radius, wheel_base)
        
        print("\n" + "=" * 60)
        print("实际测量结果:")
        print("=" * 60)
        print(f"电机最大转速: {result['max_motor_rpm']:.2f} RPM")
        print(f"达到最大转速的速度指令: {result['max_speed_cmd']}")
        print("-" * 60)
        
        print("\n速度特性分析（速度指令 vs 实际转速）:")
        print("=" * 60)
        
        regression = linear_regression(result["data_points"])
        if regression:
            print(f"\n线性回归结果:")
            print(f"  拟合方程: RPM = {regression['slope']:.6f} × speed_cmd + {regression['intercept']:.4f}")
            print(f"  相关系数 R²: {regression['r_squared']:.6f}")
            
            if regression['r_squared'] > 0.95:
                print(f"  判断: 速度指令与实际转速呈强线性关系 (R² > 0.95)")
            elif regression['r_squared'] > 0.8:
                print(f"  判断: 速度指令与实际转速呈较好的线性关系 (0.8 < R² ≤ 0.95)")
            elif regression['r_squared'] > 0.6:
                print(f"  判断: 速度指令与实际转速存在一定线性关系 (0.6 < R² ≤ 0.8)")
            else:
                print(f"  判断: 速度指令与实际转速线性关系较弱 (R² ≤ 0.6)")
            
            print(f"\n斜率含义: 每增加1单位速度指令，转速增加 {regression['slope']:.4f} RPM")
            
            if regression['intercept'] != 0:
                print(f"截距含义: 当速度指令为0时，转速约为 {regression['intercept']:.4f} RPM")
            
            print(f"\n数据点列表:")
            print(f"  {'速度指令':>10} {'实际转速':>10} {'拟合转速':>10} {'误差':>10}")
            print(f"  {'----------':>10} {'----------':>10} {'----------':>10} {'----------':>10}")
            for x, y in result["data_points"]:
                fitted = regression['slope'] * x + regression['intercept']
                error = y - fitted
                print(f"  {x:>10} {y:>10.2f} {fitted:>10.2f} {error:>10.2f}")
        else:
            print("  数据点不足，无法进行线性回归分析")
        
        max_rpm = result['max_motor_rpm']
        
        print("\n" + "=" * 60)
        print("计算过程:")
        print("=" * 60)
        
        if motion_type == "linear":
            print("\n1. 最大线速度计算:")
            print(f"   公式: v_max = (RPM_max × 2πr) / 60")
            print(f"   代入: v_max = ({max_rpm:.2f} × 2 × π × {wheel_radius}) / 60")
            max_linear_speed = (max_rpm * 2 * math.pi * wheel_radius) / 60
            print(f"   结果: v_max = {max_linear_speed:.4f} m/s")
            
            print("\n2. 最大角速度计算:")
            print(f"   公式: ω_max = (v_max × 2) / L")
            print(f"   其中: v_max = 最大线速度 = {max_linear_speed:.4f} m/s")
            print(f"         L = 轮距 = {wheel_base} m")
            print(f"   代入: ω_max = ({max_linear_speed:.4f} × 2) / {wheel_base}")
            max_angular_speed = max_linear_speed * 2 / wheel_base
            print(f"   结果: ω_max = {max_angular_speed:.4f} rad/s")
            
            print("\n3. 与配置文件对比:")
            print(f"   配置最大线速度: {base_config.max_linear_speed} m/s")
            print(f"   实际最大线速度: {max_linear_speed:.4f} m/s")
            print(f"   差异: {(max_linear_speed - base_config.max_linear_speed):+.4f} m/s")
            print(f"   偏差率: {((max_linear_speed - base_config.max_linear_speed) / base_config.max_linear_speed * 100):+.2f}%")
            
            print(f"\n   配置最大角速度: {base_config.max_angular_speed} rad/s")
            print(f"   实际最大角速度: {max_angular_speed:.4f} rad/s")
            print(f"   差异: {(max_angular_speed - base_config.max_angular_speed):+.4f} rad/s")
            print(f"   偏差率: {((max_angular_speed - base_config.max_angular_speed) / base_config.max_angular_speed * 100):+.2f}%")
            
            print("\n" + "=" * 60)
            print("最终结果:")
            print("=" * 60)
            print(f"电机最大转速: {max_rpm:.2f} RPM")
            print(f"最大线速度: {max_linear_speed:.4f} m/s")
            print(f"最大角速度(由直线推算): {max_angular_speed:.4f} rad/s")
            if regression:
                print(f"线性相关系数 R²: {regression['r_squared']:.6f}")
            print("=" * 60)
        
        else:
            print("\n1. 轮缘线速度计算:")
            print(f"   公式: v_max = (RPM_max × 2πr) / 60")
            print(f"   代入: v_max = ({max_rpm:.2f} × 2 × π × {wheel_radius}) / 60")
            max_linear_speed = (max_rpm * 2 * math.pi * wheel_radius) / 60
            print(f"   结果: v_max = {max_linear_speed:.4f} m/s")
            
            print("\n2. 最大角速度计算:")
            print(f"   公式: ω_max = (v_max × 2) / L")
            print(f"   其中: v_max = 轮缘线速度 = {max_linear_speed:.4f} m/s")
            print(f"         L = 轮距 = {wheel_base} m")
            print(f"   代入: ω_max = ({max_linear_speed:.4f} × 2) / {wheel_base}")
            max_angular_speed = max_linear_speed * 2 / wheel_base
            print(f"   结果: ω_max = {max_angular_speed:.4f} rad/s")
            
            print("\n3. 与配置文件对比:")
            print(f"   配置最大角速度: {base_config.max_angular_speed} rad/s")
            print(f"   实测最大角速度: {max_angular_speed:.4f} rad/s")
            print(f"   差异: {(max_angular_speed - base_config.max_angular_speed):+.4f} rad/s")
            print(f"   偏差率: {((max_angular_speed - base_config.max_angular_speed) / base_config.max_angular_speed * 100):+.2f}%")
            
            print(f"\n   对应线速度(参考): {max_linear_speed:.4f} m/s")
            print(f"   配置最大线速度: {base_config.max_linear_speed} m/s")
            print(f"   偏差率: {((max_linear_speed - base_config.max_linear_speed) / base_config.max_linear_speed * 100):+.2f}%")
            
            print("\n" + "=" * 60)
            print("最终结果:")
            print("=" * 60)
            print(f"电机最大转速: {max_rpm:.2f} RPM")
            print(f"最大角速度: {max_angular_speed:.4f} rad/s")
            print(f"轮缘线速度: {max_linear_speed:.4f} m/s")
            if regression:
                print(f"线性相关系数 R²: {regression['r_squared']:.6f}")
            print("=" * 60)
        
        return 0
            
    except FileNotFoundError as e:
        print(f"错误: 找不到机器人配置文件: {e}")
        print("可用的机器人配置:")
        list_robots()
        return 1
    except RuntimeError as e:
        print(f"\n错误: {e}")
        print("请检查硬件连接和串口配置")
        return 1
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        if driver:
            driver.stop()
            driver.cleanup()
            print("\n已停止电机并释放资源")


if __name__ == "__main__":
    sys.exit(main())
