# src/config_loader.py

import yaml
import os
from typing import Dict, Any, List, Optional
from dataclasses import dataclass

@dataclass
class SystemConfig:
    """系统配置"""
    log_level: str
    debug_mode: bool
    hardware_timeout: float
    max_retries: int

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
    input_size: int
    tennis_width_far: int
    tennis_width_near: int

@dataclass
class ControlConfig:
    """控制参数配置"""
    kp_dist: float
    kp_angle: float
    wheel_base: float
    max_speed: int
    min_speed_ratio: int
    idle_speed: int

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
    direction_inverted: bool

@dataclass
class PIDConfig:
    """PID控制器配置"""
    kp: float
    ki: float
    kd: float

@dataclass
class UARTConfig:
    """UART配置"""
    port: str
    baudrate: int
    ppr: int
    pwm_freq: int

@dataclass
class BaseConfig:
    """底盘配置"""
    type: str
    driver: str
    wheel_radius: float
    wheel_base: float
    max_linear_speed: float
    max_angular_speed: float
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

@dataclass
class HardwareConfig:
    """硬件配置"""
    mode: str
    model_format: str
    base: BaseConfig
    camera: CameraConfig
    arm: Optional[ArmConfig] = None

@dataclass
class DeviceParameters:
    """设备特定参数"""
    frame_width: int

@dataclass
class DeviceConfig:
    """设备配置"""
    hardware: HardwareConfig
    parameters: DeviceParameters

@dataclass
class RobotConfig:
    """机器人完整配置"""
    system: SystemConfig
    task: TaskConfig
    vision: VisionConfig
    control: ControlConfig
    device: DeviceConfig

def load_yaml_file(file_path: str) -> Dict[str, Any]:
    """加载单个YAML文件"""
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def load_common_config(config_dir: str = "config") -> Dict[str, Any]:
    """加载通用配置"""
    common_path = os.path.join(config_dir, "common.yaml")
    return load_yaml_file(common_path)

def load_robot_config(robot_name: str, config_dir: str = "config") -> Dict[str, Any]:
    """加载机器人特定配置"""
    robot_path = os.path.join(config_dir, "robots", f"{robot_name}.yaml")
    return load_yaml_file(robot_path)

def load_config(robot_name: str = "aka01b", config_dir: str = "config") -> RobotConfig:
    """加载完整配置"""
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    full_config_dir = os.path.join(base_dir, config_dir)
    
    common = load_common_config(full_config_dir)
    robot = load_robot_config(robot_name, full_config_dir)
    
    base_data = robot['HARDWARE']['BASE']
    base_type = base_data['TYPE']
    driver_type = base_data['DRIVER']
    
    motors_config = None
    pid_config = None
    uart_config = None
    
    if driver_type == "d24a_jgb37":
        motors_config = []
        for motor_data in base_data['MOTORS']:
            motors_config.append(MotorConfig(
                name=motor_data['name'],
                in2=motor_data['in2'],
                in1=motor_data['in1'],
                phase_A=motor_data['phase_A'],
                phase_B=motor_data['phase_B'],
                pwm_chip=motor_data['pwm_chip'],
                pwm_channel=motor_data['pwm_channel'],
                direction_inverted=motor_data.get('direction_inverted', False)
            ))
        pid_config = PIDConfig(
            kp=base_data['PID']['KP'],
            ki=base_data['PID']['KI'],
            kd=base_data['PID']['KD']
        )
    elif driver_type == "esp32_c3_tt":
        uart_data = base_data['UART']
        uart_config = UARTConfig(
            port=uart_data['PORT'],
            baudrate=uart_data['BAUDRATE'],
            ppr=uart_data['PPR'],
            pwm_freq=uart_data['PWM_FREQ']
        )
    
    arm_config = None
    if 'ARM' in robot['HARDWARE']:
        arm_data = robot['HARDWARE']['ARM']
        arm_config = ArmConfig(
            type=arm_data.get('TYPE', 'unknown'),
            servo_ids=arm_data.get('SERVO_IDS', []),
            max_speed=arm_data.get('MAX_SPEED', 1.0),
            home_position=arm_data.get('HOME_POSITION', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        )
    
    return RobotConfig(
        system=SystemConfig(
            log_level=common['SYSTEM']['LOG_LEVEL'],
            debug_mode=common['SYSTEM']['DEBUG_MODE'],
            hardware_timeout=common['SYSTEM']['HARDWARE_TIMEOUT'],
            max_retries=common['SYSTEM']['MAX_RETRIES']
        ),
        task=TaskConfig(
            search_timeout=common['TASK']['SEARCH_TIMEOUT'],
            pick_retry_count=common['TASK']['PICK_RETRY_COUNT'],
            place_retry_count=common['TASK']['PLACE_RETRY_COUNT'],
            bucket_approach_distance=common['TASK']['BUCKET_APPROACH_DISTANCE'],
            max_objects_per_task=common['TASK']['MAX_OBJECTS_PER_TASK']
        ),
        vision=VisionConfig(
            model_name=common['VISION']['MODEL_NAME'],
            confidence_threshold=common['VISION']['CONFIDENCE_THRESHOLD'],
            nms_threshold=common['VISION']['NMS_THRESHOLD'],
            input_size=common['VISION']['INPUT_SIZE'],
            tennis_width_far=common['VISION']['TENNIS_WIDTH_FAR'],
            tennis_width_near=common['VISION']['TENNIS_WIDTH_NEAR']
        ),
        control=ControlConfig(
            kp_dist=common['CONTROL']['KP_DIST'],
            kp_angle=common['CONTROL']['KP_ANGLE'],
            wheel_base=common['CONTROL']['WHEEL_BASE'],
            max_speed=common['CONTROL']['MAX_SPEED'],
            min_speed_ratio=common['CONTROL']['MIN_SPEED_RATIO'],
            idle_speed=common['CONTROL']['IDLE_SPEED']
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
                    max_linear_speed=base_data['MAX_LINEAR_SPEED'],
                    max_angular_speed=base_data['MAX_ANGULAR_SPEED'],
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
            ),
            parameters=DeviceParameters(
                frame_width=robot['PARAMETERS']['FRAME_WIDTH']
            )
        )
    )