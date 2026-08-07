#!/usr/bin/env python3
"""距离校准工具 - 交互式校准网球检测距离参数"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config_loader import load_config
from src.controller.vision_module import VisionModule
from src.camera.usb_camera import USBCamera
from _robot_select import select_robot


def main():
    """主程序入口"""
    robot_name = select_robot()
    if robot_name is None:
        return

    print(f"正在加载 {robot_name} 的配置...")
    config = load_config(robot_name=robot_name)
    
    camera = None
    vision = None
    
    try:
        camera = USBCamera({
            'device_id': config.device.hardware.camera.device_id,
            'resolution': config.device.hardware.camera.resolution,
            'frame_rate': config.device.hardware.camera.frame_rate
        })
        
        if not camera.is_opened():
            print("摄像头未打开，退出程序")
            return
        
        print("正在加载视觉模型...")
        vision = VisionModule(config)
        
        config_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
        
        success = vision.calibrate_distance(robot_name, camera, config_dir=config_dir)
        
        if success:
            print("\n校准完成！")
        else:
            print("\n校准失败")
            
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        if vision is not None:
            vision.release()
        if camera is not None:
            camera.release()


if __name__ == "__main__":
    main()
