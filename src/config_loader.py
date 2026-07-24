# src/config_loader.py

import yaml
import os
from typing import Dict, Any, List, Optional
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
    input_size: int
    tennis_width_far: int
    tennis_width_near: int


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
    wheel_base: float
    max_linear_speed: float
    approach_threshold: float
    target_x: float
    threshold_x: float
    target_distance: float
    threshold_d: float
    approach_kp: float
    approach_ki: float
    approach_kd: float
    approach_kp_angle: float
    search_rotation_speed: float

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
    min_pwm: int = 20
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
    statemachine: StateMachineConfig
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
            pwm_freq=uart_data['PWM_FREQ'],
            min_pwm=uart_data.get('MIN_PWM', 20),
            turn_threshold=uart_data.get('TURN_THRESHOLD', 20),
            direction_forward=uart_data.get('DIRECTION_FORWARD', 1)
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
    
    return RobotConfig(
        system=SystemConfig(
            robot_id=common['SYSTEM']['ROBOT_ID'],
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
            input_size=robot['VISION']['INPUT_SIZE'],
            tennis_width_far=robot['VISION']['TENNIS_WIDTH_FAR'],
            tennis_width_near=robot['VISION']['TENNIS_WIDTH_NEAR']
        ),
        control=ControlConfig(
            kp_dist=base_data.get('CONTROL', {}).get('KP_DIST', 0.8),
            kp_angle=base_data.get('CONTROL', {}).get('KP_ANGLE', 0.005),
            wheel_base=base_data.get('WHEEL_BASE', 0.2),
            max_linear_speed=base_data.get('CONTROL', {}).get('MAX_LINEAR_SPEED', 0.4),
            approach_threshold=base_data.get('CONTROL', {}).get('APPROACH_THRESHOLD', 0.9),
            target_x=sm_data.get('TARGET_X', 0.0),
            threshold_x=sm_data.get('THRESHOLD_X', 10.0),
            target_distance=sm_data.get('TARGET_DISTANCE', 0.2),
            threshold_d=sm_data.get('THRESHOLD_D', 0.01),
            approach_kp=base_data.get('CONTROL', {}).get('APPROACH_KP', 0.6),
            approach_ki=base_data.get('CONTROL', {}).get('APPROACH_KI', 0.01),
            approach_kd=base_data.get('CONTROL', {}).get('APPROACH_KD', 0.01),
            approach_kp_angle=base_data.get('CONTROL', {}).get('APPROACH_KP_ANGLE', 0.002),
            search_rotation_speed=base_data.get('CONTROL', {}).get('SEARCH_ROTATION_SPEED', 0.3)
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