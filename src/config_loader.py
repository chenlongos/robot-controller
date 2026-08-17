# src/config_loader.py

import yaml
import os
import copy
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass

@dataclass
class SystemConfig:
    """系统配置"""
    robot_id: str
    log_level: str
    hardware_timeout: float
    max_retries: int
    display_enabled: bool
    display_interval: int

@dataclass
class TaskConfig:
    """任务配置"""
    search_timeout: int
    pick_retry_count: int
    place_retry_count: int
    bucket_approach_distance: float
    max_objects_per_task: int

@dataclass
class VisionConfig:
    """视觉参数配置"""
    model_name: str
    confidence_threshold: float
    nms_threshold: float
    bucket_color: str


@dataclass
class StateMachineConfig:
    """状态机参数配置"""
    target_x: float
    target_distance: float
    threshold_x: float
    threshold_d: float
    grip_threshold: int
    reach_count_threshold: int
    bucket_edge_threshold: int

@dataclass
class ControlConfig:
    """控制参数配置"""
    kp_dist: float
    kp_angle: float
    ki: float
    kd: float
    wheel_base: float
    max_linear_speed: float
    max_angular_speed: float
    search_rotation_speed: float
    align_rotation_speed: float
    target_x: float
    target_distance: float
    lpf_alpha: float = 0.3
    angular_limit_near: float = 0.3
    angular_limit_far: float = 0.15
    linear_decay_factor: float = 0.7

@dataclass
class MotorConfig:
    """电机配置"""
    name: str
    in2: int
    in1: int
    phase_A: int
    phase_B: int
    pwm_chip: int
    pwm_channel: int
    direction_forward: int = 1

@dataclass
class PIDConfig:
    """PID控制器配置"""
    kp: float
    ki: float
    kd: float
    output_limit: float = 100.0
    max_rate: float = 5.0

@dataclass
class UARTConfig:
    """UART配置"""
    port: str
    baudrate: int
    ppr: int
    pwm_freq: int
    min_pwm: int = 20
    max_pwm: int = 60
    turn_threshold: int = 20
    direction_forward: int = 1

@dataclass
class BaseConfig:
    """底盘配置"""
    type: str
    driver: str
    wheel_radius: float
    wheel_base: float
    max_linear_speed: float
    max_angular_speed: float
    linear_k: float = 4.0
    rotation_k: float = 6.0
    motors: Optional[List[MotorConfig]] = None
    pid: Optional[PIDConfig] = None
    uart: Optional[UARTConfig] = None

@dataclass
class CameraConfig:
    """摄像头配置"""
    type: str
    device_id: int
    resolution: tuple
    frame_rate: int

@dataclass
class ArmConfig:
    """机械臂配置"""
    type: str
    servo_ids: List[int]
    max_speed: float
    home_position: List[float]
    port: Optional[str] = None
    baudrate: Optional[int] = None
    use_degrees: Optional[bool] = None
    disable_torque_on_disconnect: Optional[bool] = None
    max_relative_target: Optional[float] = None

@dataclass
class HardwareConfig:
    """硬件配置"""
    mode: str
    model_format: str
    base: BaseConfig
    camera: CameraConfig
    arm: Optional[ArmConfig] = None

@dataclass
class DeviceConfig:
    """设备配置"""
    hardware: HardwareConfig

@dataclass
class RobotConfig:
    """机器人完整配置"""
    system: SystemConfig
    task: TaskConfig
    vision: VisionConfig
    control: ControlConfig
    statemachine: StateMachineConfig
    device: DeviceConfig


def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """递归合并两个字典，override的值覆盖base中的值"""
    result = copy.deepcopy(base)
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def load_yaml_file(file_path: str) -> Dict[str, Any]:
    """加载单个YAML文件"""
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def load_common_config(config_dir: str = "config") -> Dict[str, Any]:
    """加载通用配置 (config/common.yaml)"""
    common_path = os.path.join(config_dir, "common.yaml")
    return load_yaml_file(common_path)


def load_robot_config(robot_name: str, config_dir: str = "config") -> Dict[str, Any]:
    """加载机器人特定配置 (config/robots/<robot_name>.yaml)"""
    robot_path = os.path.join(config_dir, "robots", f"{robot_name}.yaml")
    return load_yaml_file(robot_path)


def parse_robot_id(robot_id: str) -> Tuple[str, str]:
    """解析 ROBOT_ID 为 (robot_type, board_name)
    
    规则:
      - "aka00v4-lubancat3" -> ("aka00v4", "lubancat3")
      - "aka01b-rk3588" -> ("aka01b", "rk3588")
    
    Raises:
        ValueError: 如果 ROBOT_ID 不包含 '-' 分隔符
    """
    parts = robot_id.split("-", 1)
    if len(parts) != 2:
        raise ValueError(
            f"ROBOT_ID '{robot_id}' 格式错误，应为 '<robot_type>-<board_name>' 格式"
        )
    return parts[0], parts[1]


def load_config(robot_name: Optional[str] = None, config_dir: str = "config") -> RobotConfig:
    """加载完整配置
    
    Args:
        robot_name: 机器人ID (如 "aka00v4-lubancat3")。若为None则从 config/common.yaml 读取 ROBOT_ID。
        config_dir: 配置文件目录
    
    加载逻辑:
        1. 从 config/common.yaml 获取 ROBOT_ID (若参数未提供)
        2. 解析 ROBOT_ID: <robot_type>-<board_name>
        3. 加载 <robot_type>-common.yaml (共有配置)
        4. 加载 <robot_type>-<board_name>.yaml (开发板独有配置)
        5. 深度合并：开发板配置覆盖共有配置
        6. 构建 RobotConfig
    """
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    full_config_dir = os.path.join(base_dir, config_dir)
    
    common = load_common_config(full_config_dir)
    
    robot_id = robot_name
    if robot_id is None:
        robot_id = common['SYSTEM']['ROBOT_ID']
    
    robot_type, board_name = parse_robot_id(robot_id)
    
    config_dir_robots = os.path.join(full_config_dir, "robots")
    common_path = os.path.join(config_dir_robots, f"{robot_type}-common.yaml")
    if not os.path.exists(common_path):
        common_template = os.path.join(config_dir_robots, f"{robot_type}-common.template.yaml")
        if os.path.exists(common_template):
            common_path = common_template
        else:
            raise FileNotFoundError(f"共有配置文件不存在: {common_path}")
    board_path = os.path.join(config_dir_robots, f"{robot_type}-{board_name}.yaml")

    if not os.path.exists(board_path):
        raise FileNotFoundError(f"开发板配置文件不存在: {board_path}")
    
    merged_robot = load_yaml_file(common_path)
    board_data = load_yaml_file(board_path)
    merged_robot = deep_merge(merged_robot, board_data)
    robot = merged_robot
    
    base_data = robot['HARDWARE']['BASE']
    base_type = base_data['TYPE']
    driver_data = base_data['DRIVER']
    driver_type = driver_data['TYPE']
    
    motors_config = None
    pid_config = None
    uart_config = None
    
    if driver_type == "d24a_jgb37":
        motors_config = []
        for motor_data in base_data['MOTORS']:
            if 'direction_forward' in motor_data:
                dir_fwd = motor_data['direction_forward']
            elif 'direction_inverted' in motor_data:
                dir_fwd = -1 if motor_data['direction_inverted'] else 1
            else:
                dir_fwd = 1
            motors_config.append(MotorConfig(
                name=motor_data['name'],
                in2=motor_data['in2'],
                in1=motor_data['in1'],
                phase_A=motor_data['phase_A'],
                phase_B=motor_data['phase_B'],
                pwm_chip=motor_data['pwm_chip'],
                pwm_channel=motor_data['pwm_channel'],
                direction_forward=dir_fwd
            ))
        pid_config = PIDConfig(
            kp=driver_data.get('KP', 0.5),
            ki=driver_data.get('KI', 0.1),
            kd=driver_data.get('KD', 0.05),
            output_limit=driver_data.get('OUTPUT_LIMIT', 100.0),
            max_rate=driver_data.get('MAX_RATE', 5.0)
        )
    elif driver_type == "esp32_c3_tt":
        uart_config = UARTConfig(
            port=base_data['PORT'],
            baudrate=base_data['BAUDRATE'],
            ppr=driver_data.get('PPR', 4680),
            pwm_freq=driver_data.get('PWM_FREQ', 20000),
            min_pwm=driver_data.get('MIN_PWM', 20),
            max_pwm=driver_data.get('MAX_PWM', 60),
            turn_threshold=driver_data.get('TURN_THRESHOLD', 20),
            direction_forward=driver_data.get('DIRECTION_FORWARD', 1)
        )
        pid_config = PIDConfig(
            kp=driver_data.get('KP', 0.5),
            ki=driver_data.get('KI', 0.1),
            kd=driver_data.get('KD', 0.05),
            output_limit=driver_data.get('OUTPUT_LIMIT', 100.0),
            max_rate=driver_data.get('MAX_RATE', 5.0)
        )
    
    arm_config = None
    if 'ARM' in robot['HARDWARE']:
        arm_data = robot['HARDWARE']['ARM']
        arm_config = ArmConfig(
            type=arm_data.get('TYPE', 'unknown'),
            servo_ids=arm_data.get('SERVO_IDS', []),
            max_speed=arm_data.get('MAX_SPEED', 1.0),
            home_position=arm_data.get('HOME_POSITION', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            port=arm_data.get('PORT', None),
            baudrate=arm_data.get('BAUDRATE', None),
            use_degrees=arm_data.get('USE_DEGREES', None),
            disable_torque_on_disconnect=arm_data.get('DISABLE_TORQUE_ON_DISCONNECT', None),
            max_relative_target=arm_data.get('MAX_RELATIVE_TARGET', None)
        )
    
    sm_data = robot.get('STATEMACHINE', {})
    control_data = base_data.get('CONTROL', {})
    
    return RobotConfig(
        system=SystemConfig(
            robot_id=robot_id,
            log_level=common['SYSTEM']['LOG_LEVEL'],
            hardware_timeout=common['SYSTEM']['HARDWARE_TIMEOUT'],
            max_retries=common['SYSTEM']['MAX_RETRIES'],
            display_enabled=common['SYSTEM'].get('DISPLAY_ENABLED', False),
            display_interval=common['SYSTEM'].get('DISPLAY_INTERVAL', 5)
        ),
        task=TaskConfig(
            search_timeout=common['TASK']['SEARCH_TIMEOUT'],
            pick_retry_count=common['TASK']['PICK_RETRY_COUNT'],
            place_retry_count=common['TASK']['PLACE_RETRY_COUNT'],
            bucket_approach_distance=common['TASK']['BUCKET_APPROACH_DISTANCE'],
            max_objects_per_task=common['TASK']['MAX_OBJECTS_PER_TASK']
        ),
        vision=VisionConfig(
            model_name=robot['VISION']['MODEL_NAME'],
            confidence_threshold=robot['VISION']['CONFIDENCE_THRESHOLD'],
            nms_threshold=robot['VISION']['NMS_THRESHOLD'],
            bucket_color=robot['VISION']['BUCKET_COLOR']
        ),
        control=ControlConfig(
            kp_dist=control_data.get('KP_DIST', 0.8),
            kp_angle=control_data.get('KP_ANGLE', 0.005),
            ki=control_data.get('KI', 0.5),
            kd=control_data.get('KD', 0.1),
            wheel_base=base_data.get('WHEEL_BASE', 0.2),
            max_linear_speed=control_data.get('MAX_LINEAR_SPEED', 0.4),
            max_angular_speed=control_data.get('MAX_ANGULAR_SPEED', 1.0),
            search_rotation_speed=control_data.get('SEARCH_ROTATION_SPEED', 0.3),
            align_rotation_speed=control_data.get('ALIGN_ROTATION_SPEED', 0.3),
            target_x=sm_data.get('TARGET_X', 0.0),
            target_distance=sm_data.get('TARGET_DISTANCE', 0.2),
            lpf_alpha=control_data.get('LPF_ALPHA', 0.3),
            angular_limit_near=control_data.get('ANGULAR_LIMIT_NEAR', 0.3),
            angular_limit_far=control_data.get('ANGULAR_LIMIT_FAR', 0.15),
            linear_decay_factor=control_data.get('LINEAR_DECAY_FACTOR', 0.7)
        ),
        statemachine=StateMachineConfig(
            target_x=sm_data.get('TARGET_X', 0.0),
            target_distance=sm_data.get('TARGET_DISTANCE', 0.2),
            threshold_x=sm_data.get('THRESHOLD_X', 50.0),
            threshold_d=sm_data.get('THRESHOLD_D', 0.01),
            grip_threshold=sm_data.get('GRIP_THRESHOLD', 90),
            reach_count_threshold=sm_data.get('REACH_COUNT_THRESHOLD', 10),
            bucket_edge_threshold=sm_data.get('BUCKET_EDGE_THRESHOLD', 20)
        ),
        device=DeviceConfig(
            hardware=HardwareConfig(
                mode=robot['HARDWARE']['MODE'],
                model_format=robot['HARDWARE']['MODEL_FORMAT'],
                base=BaseConfig(
                    type=base_type,
                    driver=driver_type,
                    wheel_radius=base_data['WHEEL_RADIUS'],
                    wheel_base=base_data['WHEEL_BASE'],
                    max_linear_speed=base_data.get('CONTROL', {}).get('MAX_LINEAR_SPEED', 0.4),
                    max_angular_speed=base_data.get('CONTROL', {}).get('MAX_ANGULAR_SPEED', 1.0),
                    linear_k=driver_data.get('LINEAR_K', 4.0),
                    rotation_k=driver_data.get('ROTATION_K', 6.0),
                    motors=motors_config,
                    pid=pid_config,
                    uart=uart_config
                ),
                camera=CameraConfig(
                    type=robot['HARDWARE']['CAMERA']['TYPE'],
                    device_id=robot['HARDWARE']['CAMERA']['DEVICE_ID'],
                    resolution=tuple(robot['HARDWARE']['CAMERA']['RESOLUTION']),
                    frame_rate=robot['HARDWARE']['CAMERA']['FRAME_RATE']
                ),
                arm=arm_config
            )
        )
    )