"""机械臂测试程序"""
import sys
import os
import time
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config_loader import load_config


def create_arm(config) -> object:
    """根据配置创建机械臂实例"""
    arm_config = config.device.hardware.arm
    
    arm_type = arm_config.type
    logger.info(f"机械臂类型: {arm_type}")
    
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
    global logger
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
        
        logger.info("执行抓取动作...")
        arm.grab()
        logger.info("抓取完成")
        
        time.sleep(2)
        
        logger.info("执行释放动作...")
        arm.release()
        logger.info("释放完成")
        
        time.sleep(1)
        
        logger.info("测试完成")
        
    except Exception as e:
        logger.error(f"测试失败: {e}", exc_info=True)
        raise
    finally:
        if arm:
            logger.info("断开机械臂连接...")
            arm.disconnect()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='机械臂测试程序')
    parser.add_argument('--robot', type=str, default='aka00v4-rk3576', 
                        help='机器人名称 (默认: aka00v4-rk3576)')
    args = parser.parse_args()
    
    test_arm(args.robot)