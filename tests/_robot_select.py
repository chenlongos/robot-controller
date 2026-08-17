"""测试脚本共享的机器人选择工具。

从 config/robots 目录扫描可用机器人配置，并提供交互式选择。
各测试脚本通过 `from _robot_select import list_robots, select_robot` 复用。
"""

import os


def list_robots():
    """扫描 config/robots 目录，返回可用机器人名称列表（去除 .yaml 后缀）。
    
    过滤掉 *-common.yaml 共有配置文件，只返回可选择的开发板配置。
    """
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config", "robots"
    )
    robots = []
    if os.path.isdir(config_dir):
        for f in sorted(os.listdir(config_dir)):
            if f.endswith('.yaml') and not f.endswith('-common.yaml') and not f.endswith('template.yaml'):
                robots.append(f[:-5])
    return robots


def select_robot():
    """交互式选择机器人，返回机器人名称；用户取消时返回 None。"""
    robots = list_robots()
    if not robots:
        print("错误: 未找到任何机器人配置")
        return None

    print("\n可用的机器人列表:")
    print("-" * 40)
    for i, name in enumerate(robots, 1):
        print(f"  {i}. {name}")
    print("-" * 40)

    while True:
        try:
            choice = input(f"\n请选择机器人 (1-{len(robots)}，默认1): ").strip()
            if choice == "":
                idx = 0
            else:
                idx = int(choice) - 1
            if 0 <= idx < len(robots):
                return robots[idx]
            else:
                print(f"请输入 1 到 {len(robots)} 之间的数字")
        except ValueError:
            print("请输入有效的数字")
        except (EOFError, KeyboardInterrupt):
            print("\n已取消")
            return None
