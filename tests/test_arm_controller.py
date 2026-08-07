"""机械臂控制器测试程序"""
import sys
import os
import argparse
import ast

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config_loader import load_config
from src.controller.arm_controller import ArmController


def create_arm(config) -> object:
    """根据配置创建机械臂实例"""
    arm_config = config.device.hardware.arm
    
    arm_type = arm_config.type
    
    if arm_type == "zp10s":
        from src.arm.zp10s_arm import ZP10SArm
        arm = ZP10SArm({
            'port': arm_config.port,
            'baudrate': arm_config.baudrate,
            'timeout': 0.1
        })
    elif arm_type == "so101":
        from src.arm.so101_arm import SO101Arm
        arm = SO101Arm({
            'port': arm_config.port,
            'use_degrees': arm_config.use_degrees,
            'disable_torque_on_disconnect': arm_config.disable_torque_on_disconnect,
            'max_relative_target': arm_config.max_relative_target
        })
    else:
        raise ValueError(f"不支持的机械臂类型: {arm_type}")
    
    return arm


def test_arm_controller(robot_name: str):
    """测试机械臂控制器"""
    import logging
    
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s"
    )
    logger = logging.getLogger(__name__)
    
    logger.info(f"加载 {robot_name} 配置...")
    config = load_config(robot_name=robot_name)
    
    arm = None
    controller = None
    
    try:
        arm = create_arm(config)
        
        logger.info("连接机械臂...")
        arm.connect()
        
        controller = ArmController(arm, robot_name=robot_name)
        
        logger.info("进入控制器测试模式，输入 exit 退出")
        
        while True:
            print("\n" + "="*50)
            print("可用操作:")
            print("  1. 设置动作序列 (set)")
            print("  2. 执行动作序列 (exec)")
            print("  3. 停止机械臂 (stop)")
            print("  4. 退出 (exit)")
            
            user_input = input("\n请输入操作编号: ")
            
            if user_input.strip().lower() == "exit" or user_input == "4":
                logger.info("退出测试")
                break
            
            if user_input == "1" or user_input.strip().lower() == "set":
                print("\n设置动作序列")
                print("可用动作名称: origin, pick, put, open_gripper, close_gripper")
                
                action_name = input("请输入动作名称: ").strip()
                
                if action_name not in ArmController.ALLOWED_ACTIONS:
                    print(f"错误: 不支持的动作名称 '{action_name}'")
                    continue
                
                print(f"\n为动作 '{action_name}' 添加步骤")
                print("步骤类型:")
                print("  joint - 关节位置控制，格式: positions [x1, x2, ...]")
                print("  gripper - 夹爪控制，格式: position 角度值（如90、150）")
                print("  delay - 延时，格式: duration 秒数")
                print("输入 'done' 完成步骤添加")
                
                sequence = []
                
                while True:
                    step_input = input("\n请输入步骤 (格式: type 参数): ").strip()
                    
                    if step_input.lower() == "done":
                        break
                    
                    parts = step_input.split()
                    if len(parts) < 2:
                        print("错误: 请输入格式为 'type 参数'")
                        continue
                    
                    step_type = parts[0]
                    params = " ".join(parts[1:])
                    
                    if step_type == "joint":
                        try:
                            positions = ast.literal_eval(params)
                            if not isinstance(positions, list):
                                raise ValueError("positions必须是列表")
                            sequence.append({"type": "joint", "positions": positions})
                            print(f"添加步骤: joint positions={positions}")
                        except Exception as e:
                            print(f"错误: {e}")
                    
                    elif step_type == "gripper":
                        try:
                            position = float(params)
                            if position < 0 or position > 270:
                                raise ValueError("position必须在0-270之间")
                            sequence.append({"type": "gripper", "position": position})
                            print(f"添加步骤: gripper position={position}")
                        except Exception as e:
                            print(f"错误: {e}")
                    
                    elif step_type == "delay":
                        try:
                            duration = float(params)
                            if duration < 0:
                                raise ValueError("duration必须大于等于0")
                            sequence.append({"type": "delay", "duration": duration})
                            print(f"添加步骤: delay duration={duration}s")
                        except Exception as e:
                            print(f"错误: {e}")
                    
                    else:
                        print(f"错误: 不支持的步骤类型 '{step_type}'")
                
                try:
                    controller.set_action_sequence(action_name, sequence)
                    print(f"动作序列 '{action_name}' 设置成功")
                except Exception as e:
                    print(f"设置失败: {e}")
            
            elif user_input == "2" or user_input.strip().lower() == "exec":
                print("\n执行动作序列")
                print("可用动作名称: origin, pick, put, open_gripper, close_gripper")
                
                action_name = input("请输入要执行的动作名称: ").strip()
                
                if action_name not in ArmController.ALLOWED_ACTIONS:
                    print(f"错误: 不支持的动作名称 '{action_name}'")
                    continue
                
                try:
                    result = controller.execute_action_sequence(action_name)
                    print(f"执行结果: {result}")
                except Exception as e:
                    print(f"执行失败: {e}")
            
            elif user_input == "3" or user_input.strip().lower() == "stop":
                try:
                    controller.stop()
                    print("机械臂已停止")
                except Exception as e:
                    print(f"停止失败: {e}")
            
            else:
                print("错误: 无效的操作编号")
        
    except Exception as e:
        logger.error(f"测试失败: {e}", exc_info=True)
        raise
    finally:
        if arm:
            logger.info("断开机械臂连接...")
            arm.disconnect()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='机械臂控制器测试程序')
    parser.add_argument('--robot', type=str, default='aka00v4-lubancat3', 
                        help='机器人名称 (默认: aka00v4-rock4d)')
    args = parser.parse_args()
    
    test_arm_controller(args.robot)
