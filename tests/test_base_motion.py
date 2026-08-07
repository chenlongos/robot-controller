# tests/hardware/test_base_motion.py
"""底盘运动测试程序 - 交互式测试底盘运动功能"""

import time
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config_loader import load_config
from src.abstract.base_factory import BaseFactory
import src.base
from _robot_select import select_robot


class BaseMotionTest:
    """底盘运动测试类"""

    def __init__(self, robot_name: str):
        """初始化测试"""
        self.config = load_config(robot_name)
        self.base = None
        self._init_base()
        self.running = True
        
        self.motion_options = [
            ("w", "前进", lambda: self.base.move(0.3, 0.0, 0.0)),
            ("s", "后退", lambda: self.base.move(-0.3, 0.0, 0.0)),
            ("a", "左转", lambda: self.base.move(0.0, 0.0, 0.8)),
            ("d", "右转", lambda: self.base.move(0.0, 0.0, -0.8)),
            ("wa", "前进左转", lambda: self.base.move(0.2, 0.0, 0.5)),
            ("wd", "前进右转", lambda: self.base.move(0.2, 0.0, -0.5)),
            ("sa", "后退左转", lambda: self.base.move(-0.2, 0.0, 0.5)),
            ("sd", "后退右转", lambda: self.base.move(-0.2, 0.0, -0.5)),
            ("stop", "停止", lambda: self.base.stop()),
            ("exit", "退出", lambda: self._exit()),
        ]
    
    def _init_base(self):
        """初始化底盘"""
        try:
            base_config = self.config.device.hardware.base
            base_type = base_config.type
            self.base = BaseFactory.create_base(base_type, base_config)
            print(f"底盘初始化成功! 类型: {base_type}, 驱动: {base_config.driver}")
        except Exception as e:
            print(f"底盘初始化失败: {e}")
            sys.exit(1)
    
    def _exit(self):
        """退出测试"""
        self.running = False
        self.base.stop()
        self.base.cleanup()
    
    def show_menu(self):
        """显示运动选项菜单"""
        print("\n" + "=" * 50)
        print("底盘运动测试程序")
        print("=" * 50)
        print("可用运动方式:")
        for key, desc, _ in self.motion_options:
            print(f"  {key}: {desc}")
        print("=" * 50)
    
    def run(self):
        """运行测试主循环"""
        print("底盘运动测试程序启动...")
        print("按 Ctrl+C 或输入 exit 退出")
        
        while self.running:
            try:
                self.show_menu()
                
                choice = input("\n请选择运动方式: ").strip().lower()
                
                action = None
                desc = None
                for key, d, a in self.motion_options:
                    if choice == key:
                        action = a
                        desc = d
                        break
                
                if action is None:
                    print(f"无效的选择: {choice}")
                    continue
                
                if choice == "stop":
                    action()
                    print("底盘已停止")
                    continue
                
                if choice == "exit":
                    action()
                    print("测试程序退出")
                    break
                
                try:
                    duration = float(input(f"请输入{desc}时间（秒）: "))
                except ValueError:
                    print("无效的时间输入")
                    continue
                
                print(f"开始{desc}，持续 {duration} 秒...")
                action()
                
                start_time = time.time()
                while time.time() - start_time < duration and self.running:
                    time.sleep(0.1)
                
                self.base.stop()
                print(f"{desc}完成，已停止")
                
            except KeyboardInterrupt:
                print("\n收到中断信号")
                self._exit()
                break
            except Exception as e:
                print(f"发生错误: {e}")
                self.base.stop()


if __name__ == "__main__":
    robot_name = select_robot()
    if robot_name is None:
        sys.exit(0)

    test = BaseMotionTest(robot_name)
    test.run()