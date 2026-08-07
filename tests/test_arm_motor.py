"""机械臂测试程序"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config_loader import load_config
from _robot_select import select_robot


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


def test_arm(robot_name: str):
    """测试机械臂"""
    import logging
    
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s"
    )
    logger = logging.getLogger(__name__)
    
    logger.info(f"加载 {robot_name} 配置...")
    config = load_config(robot_name=robot_name)
    
    arm = None
    
    try:
        arm = create_arm(config)
        
        logger.info("连接机械臂...")
        arm.connect()
        
        logger.info("进入交互式测试模式，输入 exit 退出")
        
        while True:
            print("\n" + "="*50)

            try:
                joint_positions = arm.get_joint_positions()
            except Exception as e:
                print(f"读取关节位置失败: {e}")
                print("  （注意：这里不再显示默认角度以免误判，可重试读取）")
                joint_positions = None

            print("当前关节位置:")
            if joint_positions is None:
                for name in arm.JOINT_NAMES:
                    print(f"  {name}: <读取失败，无效>")
            else:
                for name, pos in zip(arm.JOINT_NAMES, joint_positions):
                    print(f"  {name}: {pos}")

            print("\n可用关节:")
            for i, joint in enumerate(arm.JOINT_NAMES):
                print(f"  {i+1}. {joint}")

            user_input = input("\n请输入要操作的关节编号和目标位置（格式: 编号 位置，或输入 exit 退出）: ")

            if user_input.strip().lower() == "exit":
                logger.info("退出测试")
                break

            try:
                parts = user_input.strip().split()
                if len(parts) != 2:
                    print("错误: 请输入格式为 '编号 位置'")
                    continue

                joint_index = int(parts[0]) - 1
                target_position = float(parts[1])

                if joint_index < 0 or joint_index >= len(arm.JOINT_NAMES):
                    print(f"错误: 关节编号必须在 1-{len(arm.JOINT_NAMES)} 之间")
                    continue

                selected_joint = arm.JOINT_NAMES[joint_index]

                try:
                    joint_positions = list(arm.get_joint_positions())
                except Exception as e:
                    print(f"读取当前关节位置失败，无法安全合并关节状态: {e}")
                    continue
                joint_positions[joint_index] = target_position
                print(f"设置关节 {selected_joint} 位置: {target_position}")
                arm.move_to_joint_positions(joint_positions)

                print("移动完成")

            except ValueError as e:
                print(f"错误: {e}")
            except Exception as e:
                print(f"操作失败: {e}")
        
    except Exception as e:
        logger.error(f"测试失败: {e}", exc_info=True)
        raise
    finally:
        if arm:
            logger.info("断开机械臂连接...")
            arm.disconnect()


if __name__ == '__main__':
    robot_name = select_robot()
    if robot_name is None:
        sys.exit(0)

    test_arm(robot_name)