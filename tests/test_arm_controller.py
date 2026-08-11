"""机械臂控制器测试程序"""
import sys
import os
import ast
import json
from pathlib import Path

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from src.config_loader import load_config
from src.controller.arm_controller import ArmController
from _robot_select import select_robot

CONFIG_DIR = os.path.join(PROJECT_ROOT, "config")


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


# ===== 配置文件转换（arm_angles <-> arm_action_sequences）=====
# 动作 -> 角度键序列的映射（双向转换依据）
#   ("joint",   "prepare")  -> joint 步骤，positions 取自 servo{i}_prepare
#   ("joint",   "lift")     -> joint 步骤，positions 取自 servo{i}_lift
#   ("gripper", "approach") -> gripper 步骤，position 取自 servo2_approach
#   ("gripper", "grab")     -> gripper 步骤，position 取自 servo2_grab
ACTION_ANGLE_MAPPING = {
    "origin":        [("joint", "lift")],
    "pick":          [("gripper", "approach"), ("joint", "prepare"),
                      ("gripper", "grab"), ("joint", "lift")],
    "put":           [("joint", "lift"), ("gripper", "approach"), ("joint", "lift")],
    "open_gripper":  [("gripper", "approach")],
    "close_gripper": [("gripper", "grab")],
}

# angles 文件键的固定输出顺序（与现有文件保持一致）
_ANGLES_KEY_ORDER = [
    "servo0_prepare", "servo1_prepare", "servo2_prepare",
    "servo2_approach", "servo2_grab",
    "servo0_lift", "servo1_lift", "servo2_lift",
]

# gripper 步骤之后插入的延时（秒）；仅在 gripper 步骤非末步时插入
DELAY_AFTER_GRIPPER = 1.0


def _angles_path(robot_type: str) -> Path:
    return Path(CONFIG_DIR) / f"arm_angles_{robot_type}.json"


def _sequences_path(robot_type: str) -> Path:
    return Path(CONFIG_DIR) / f"arm_action_sequences_{robot_type}.json"


def _joint_positions_from_angles(angles: dict, pose: str) -> list:
    """按 servo0,1,2... 顺序从 angles 中取出 servo{i}_{pose} 角度"""
    positions = []
    i = 0
    while f"servo{i}_{pose}" in angles:
        positions.append(angles[f"servo{i}_{pose}"])
        i += 1
    if not positions:
        raise KeyError(f"arm_angles 中缺少 servo*_{pose} 角度")
    return positions


def angles_to_sequences(robot_type: str = "aka00v4",
                        delay_after_gripper: float = DELAY_AFTER_GRIPPER,
                        overwrite: bool = True) -> dict:
    """根据 arm_angles_{robot_type}.json 生成 arm_action_sequences_{robot_type}.json

    映射规则见 ACTION_ANGLE_MAPPING；每个 gripper 步骤之后若还有后续步骤，
    则插入一个 delay 步骤（时长由 delay_after_gripper 指定）。

    Args:
        robot_type: 机器人类型（如 "aka00v4"）
        delay_after_gripper: gripper 步骤后插入的延时秒数，<=0 时不插入
        overwrite: True 则覆盖目标文件；False 时目标已存在则抛 FileExistsError

    Returns:
        生成的 sequences 字典
    """
    angles_path = _angles_path(robot_type)
    sequences_path = _sequences_path(robot_type)

    if not angles_path.exists():
        raise FileNotFoundError(f"角度配置文件不存在: {angles_path}")
    if not overwrite and sequences_path.exists():
        raise FileExistsError(f"目标文件已存在且 overwrite=False: {sequences_path}")

    with open(angles_path, "r", encoding="utf-8") as f:
        angles = json.load(f)

    sequences = {}
    for action, tokens in ACTION_ANGLE_MAPPING.items():
        raw_steps = []
        for kind, pose in tokens:
            if kind == "joint":
                raw_steps.append({
                    "type": "joint",
                    "positions": _joint_positions_from_angles(angles, pose),
                })
            else:  # gripper
                key = f"servo2_{pose}"
                if key not in angles:
                    raise KeyError(f"arm_angles 中缺少 {key}")
                raw_steps.append({"type": "gripper", "position": angles[key]})

        # 在 gripper 步骤后插入 delay（仅当其后还有步骤）
        steps = []
        for idx, step in enumerate(raw_steps):
            steps.append(step)
            if (step["type"] == "gripper" and delay_after_gripper > 0
                    and idx != len(raw_steps) - 1):
                steps.append({"type": "delay", "duration": delay_after_gripper})
        sequences[action] = steps

    with open(sequences_path, "w", encoding="utf-8") as f:
        json.dump(sequences, f, indent=2, ensure_ascii=False)

    print(f"[angles→sequences] 已写入 {sequences_path}")
    return sequences


def sequences_to_angles(robot_type: str = "aka00v4",
                        overwrite: bool = True) -> dict:
    """从 arm_action_sequences_{robot_type}.json 提取命名角度写回 arm_angles_{robot_type}.json

    按 ACTION_ANGLE_MAPPING 把每个动作的步骤（跳过 delay）对应到角度键。
    同一角度键被多个步骤引用时，以最先出现的值为准，后续冲突值会打印告警。

    Args:
        robot_type: 机器人类型
        overwrite: True 则覆盖目标文件；False 时目标已存在则抛 FileExistsError

    Returns:
        提取出的 angles 字典（按 _ANGLES_KEY_ORDER 排序）
    """
    angles_path = _angles_path(robot_type)
    sequences_path = _sequences_path(robot_type)

    if not sequences_path.exists():
        raise FileNotFoundError(f"动作序列文件不存在: {sequences_path}")
    if not overwrite and angles_path.exists():
        raise FileExistsError(f"目标文件已存在且 overwrite=False: {angles_path}")

    with open(sequences_path, "r", encoding="utf-8") as f:
        sequences = json.load(f)

    angles = {}

    def _set_angle(key: str, val, action: str):
        if key not in angles:
            angles[key] = val
        elif angles[key] != val:
            print(f"[warn] 角度冲突: {key} 已有 {angles[key]}，"
                  f"动作 '{action}' 提供了不同的值 {val}（保留原值）")
        # 值相等时保留先出现的值（含类型），不覆盖

    for action, tokens in ACTION_ANGLE_MAPPING.items():
        seq = sequences.get(action, [])
        if not seq:
            print(f"[warn] 动作 '{action}' 在 sequences 中为空，跳过")
            continue
        # 跳过 delay 步骤后与 tokens 对齐
        non_delay = [s for s in seq if s.get("type") != "delay"]
        if len(non_delay) != len(tokens):
            print(f"[warn] 动作 '{action}' 步骤数({len(non_delay)})与映射({len(tokens)})不符，跳过")
            continue
        for (kind, pose), step in zip(tokens, non_delay):
            kind_in_file = step.get("type")
            if kind == "joint":
                if kind_in_file != "joint":
                    print(f"[warn] 动作 '{action}' 期望 joint 步骤，实际 {kind_in_file}，跳过该步")
                    continue
                for i, val in enumerate(step.get("positions", [])):
                    _set_angle(f"servo{i}_{pose}", val, action)
            else:  # gripper
                if kind_in_file != "gripper":
                    print(f"[warn] 动作 '{action}' 期望 gripper 步骤，实际 {kind_in_file}，跳过该步")
                    continue
                _set_angle(f"servo2_{pose}", step.get("position"), action)

    # 按固定顺序输出
    ordered = {}
    for key in _ANGLES_KEY_ORDER:
        if key in angles:
            ordered[key] = angles[key]
    for key in sorted(angles):
        if key not in ordered:
            ordered[key] = angles[key]

    with open(angles_path, "w", encoding="utf-8") as f:
        json.dump(ordered, f, indent=2, ensure_ascii=False)

    print(f"[sequences→angles] 已写入 {angles_path}")
    return ordered


def convert_arm_config_menu(robot_type: str = "aka00v4") -> None:
    """交互式配置文件转换菜单（不需要连接机械臂）"""
    while True:
        print("\n" + "=" * 50)
        print(f"配置文件转换 (robot_type={robot_type})")
        print("  1. angles -> sequences")
        print("  2. sequences -> angles")
        print("  3. 返回")
        choice = input("请选择: ").strip()
        try:
            if choice == "1":
                angles_to_sequences(robot_type)
            elif choice == "2":
                sequences_to_angles(robot_type)
            elif choice == "3":
                break
            else:
                print("无效选择")
        except Exception as e:
            print(f"转换失败: {e}")


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


def main():
    """程序入口：顶层菜单，转换功能无需连接机械臂"""
    robot_name = select_robot()
    if robot_name is None:
        sys.exit(0)
    robot_type = robot_name.split("-")[0]

    while True:
        print("\n" + "=" * 50)
        print(f"机械臂测试主菜单 ({robot_name})")
        print("  1. 控制器测试 (需连接机械臂)")
        print("  2. 转换配置文件 (angles <-> sequences)")
        print("  3. 退出")
        choice = input("请选择: ").strip()
        if choice == "1":
            test_arm_controller(robot_name)
        elif choice == "2":
            convert_arm_config_menu(robot_type)
        elif choice == "3":
            print("退出")
            break
        else:
            print("无效选择")


if __name__ == '__main__':
    main()
