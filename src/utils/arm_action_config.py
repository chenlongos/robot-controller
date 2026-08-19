# src/arm_action_config.py
"""机械臂动作序列配置转换模块

提供 arm_angles_*.json <-> arm_action_sequences_*.json 的双向转换。
arm_angles 文件以具名角度键（如 servo0_prepare、servo2_grab）存储，
arm_action_sequences 文件以动作步骤（joint/gripper/delay）存储。

映射规则见 ACTION_ANGLE_MAPPING。
"""

import json
import logging
import os
from pathlib import Path
from typing import Dict, List

logger = logging.getLogger(__name__)

# angles 配置文件位置的环境变量名，供外部仓库指定自己的 arm_angles 文件，
# 设置后优先于本仓库 config/ 下的同名文件
ANGLES_PATH_ENV = "AKA00_ARM_ANGLES"

# 动作 -> 角度键序列的映射（双向转换依据）
#   ("joint",   "prepare")  -> joint 步骤，positions 取自 servo{i}_prepare
#   ("joint",   "lift")     -> joint 步骤，positions 取自 servo{i}_lift
#   ("gripper", "approach") -> gripper 步骤，position 取自 servo2_approach
#   ("gripper", "grab")     -> gripper 步骤，position 取自 servo2_grab
ACTION_ANGLE_MAPPING: Dict[str, List[tuple]] = {
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
DELAY_AFTER_GRIPPER = 0.5


def _project_root() -> Path:
    """返回项目根目录（src/utils/ 的上两级）"""
    return Path(__file__).resolve().parent.parent.parent


def _angles_path(robot_type: str) -> Path:
    """返回 arm_angles_{robot_type}.json 的路径。

    查找优先级：
    1. 环境变量 ANGLES_PATH_ENV（AKA00_ARM_ANGLES）指定的文件位置
    2. <项目根>/config/arm_angles_{robot_type}.json
    3. <项目根>/config/arm_angles_{robot_type}.template.json
    """
    env_value = os.environ.get(ANGLES_PATH_ENV, "").strip()
    if env_value:
        path = Path(env_value).expanduser()
        if not path.exists():
            logger.warning(f"环境变量 {ANGLES_PATH_ENV} 指定的文件不存在: {path}")
        else:
            logger.debug(f"arm_angles 使用环境变量 {ANGLES_PATH_ENV} 指定的路径: {path}")
        return path

    primary = _project_root() / "config" / f"arm_angles_{robot_type}.json"
    if primary.exists():
        return primary
    template = _project_root() / "config" / f"arm_angles_{robot_type}.template.json"
    return template


def _sequences_path(robot_type: str) -> Path:
    return _project_root() / "config" / f"arm_action_sequences_{robot_type}.json"


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

    sequences: Dict[str, List[Dict]] = {}
    for action, tokens in ACTION_ANGLE_MAPPING.items():
        raw_steps: List[Dict] = []
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
        steps: List[Dict] = []
        for idx, step in enumerate(raw_steps):
            steps.append(step)
            if (step["type"] == "gripper" and delay_after_gripper > 0
                    and idx != len(raw_steps) - 1):
                steps.append({"type": "delay", "duration": delay_after_gripper})
        sequences[action] = steps

    with open(sequences_path, "w", encoding="utf-8") as f:
        json.dump(sequences, f, indent=2, ensure_ascii=False)

    logger.info(f"[angles→sequences] 已写入 {sequences_path}")
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

    angles: Dict[str, float] = {}

    def _set_angle(key: str, val, action: str):
        if key not in angles:
            angles[key] = val
        elif angles[key] != val:
            logger.warning(f"角度冲突: {key} 已有 {angles[key]}，"
                           f"动作 '{action}' 提供了不同的值 {val}（保留原值）")

    for action, tokens in ACTION_ANGLE_MAPPING.items():
        seq = sequences.get(action, [])
        if not seq:
            logger.warning(f"动作 '{action}' 在 sequences 中为空，跳过")
            continue
        # 跳过 delay 步骤后与 tokens 对齐
        non_delay = [s for s in seq if s.get("type") != "delay"]
        if len(non_delay) != len(tokens):
            logger.warning(f"动作 '{action}' 步骤数({len(non_delay)})与映射({len(tokens)})不符，跳过")
            continue
        for (kind, pose), step in zip(tokens, non_delay):
            kind_in_file = step.get("type")
            if kind == "joint":
                if kind_in_file != "joint":
                    logger.warning(f"动作 '{action}' 期望 joint 步骤，实际 {kind_in_file}，跳过该步")
                    continue
                for i, val in enumerate(step.get("positions", [])):
                    _set_angle(f"servo{i}_{pose}", val, action)
            else:  # gripper
                if kind_in_file != "gripper":
                    logger.warning(f"动作 '{action}' 期望 gripper 步骤，实际 {kind_in_file}，跳过该步")
                    continue
                _set_angle(f"servo2_{pose}", step.get("position"), action)

    # 按固定顺序输出
    ordered: Dict[str, float] = {}
    for key in _ANGLES_KEY_ORDER:
        if key in angles:
            ordered[key] = angles[key]
    for key in sorted(angles):
        if key not in ordered:
            ordered[key] = angles[key]

    with open(angles_path, "w", encoding="utf-8") as f:
        json.dump(ordered, f, indent=2, ensure_ascii=False)

    logger.info(f"[sequences→angles] 已写入 {angles_path}")
    return ordered
