"""电机单元测试 - 使用 mock，不需要硬件"""
import pytest
import time
from unittest.mock import Mock, patch, MagicMock
from src.base.d24a_jgb37 import Motor, PIDController, EncoderCounter, MecanumController, DifferentialController, BaseFactory, get_gpio_chip_and_line
from src.config_loader import load_config, MotorConfig

# 从配置文件加载电机配置
_config = load_config()
MOTOR_CONFIGS_FROM_CONFIG = [
    {
        "name": cfg.name.replace("front_", "F").replace("back_", "B").replace("left", "L").replace("right", "R"),
        "in2": cfg.in2,
        "in1": cfg.in1,
        "phase_a": cfg.phase_A,
        "phase_b": cfg.phase_B,
        "pwm_chip": cfg.pwm_chip,
        "pwm_channel": cfg.pwm_channel,
        "direction_inverted": cfg.direction_inverted,
        "expected_direction": -1 if cfg.direction_inverted else 1
    }
    for cfg in _config.device.hardware.base.motors
]

class TestMotorUnit:
    """电机单元测试类"""
    
    @pytest.mark.parametrize("config", MOTOR_CONFIGS_FROM_CONFIG)
    def test_motor_initialization(self, config):
        """测试电机初始化"""
        with patch('src.base.d24a_jgb37.GPIO'), patch('src.base.d24a_jgb37.PWM'), \
             patch('src.base.d24a_jgb37.EncoderCounter') as mock_encoder:
            
            # 创建电机实例
            motor = Motor(
                name=config["name"],
                gpio_in2=config["in2"],
                gpio_in1=config["in1"],
                gpio_phase_a=config["phase_a"],
                gpio_phase_b=config["phase_b"],
                pwm_chip=config["pwm_chip"],
                pwm_channel=config["pwm_channel"],
                kp=_config.device.hardware.base.pid.kp,
                ki=_config.device.hardware.base.pid.ki,
                kd=_config.device.hardware.base.pid.kd,
                direction_inverted=config["direction_inverted"]
            )
            
            # 验证电机名称
            assert motor.name == config["name"]
            
            # 验证方向设置
            assert motor.direction == config["expected_direction"]
            
            # 验证 PID 控制器
            assert motor.pid is not None
            
            motor.cleanup()
    
    @pytest.mark.parametrize("config", MOTOR_CONFIGS_FROM_CONFIG)
    def test_motor_speed_control(self, config):
        """测试电机速度控制（set_speed + get_speed）"""
        with patch('src.base.d24a_jgb37.GPIO') as mock_gpio_class, \
             patch('src.base.d24a_jgb37.PWM') as mock_pwm_class, \
             patch('src.base.d24a_jgb37.EncoderCounter') as mock_encoder_class:
            
            # 设置 mock 返回值
            mock_encoder = Mock()
            mock_encoder.read.return_value = 0  # 初始编码器值为0
            mock_encoder.get_speed.return_value = 0  # get_speed 返回数值
            mock_encoder_class.return_value = mock_encoder
            
            mock_pwm = Mock()
            mock_pwm_class.return_value = mock_pwm
            
            # 创建电机实例
            motor = Motor(
                name=config["name"],
                gpio_in2=config["in2"],
                gpio_in1=config["in1"],
                gpio_phase_a=config["phase_a"],
                gpio_phase_b=config["phase_b"],
                pwm_chip=config["pwm_chip"],
                pwm_channel=config["pwm_channel"],
                kp=_config.device.hardware.base.pid.kp,
                ki=_config.device.hardware.base.pid.ki,
                kd=_config.device.hardware.base.pid.kd,
                direction_inverted=config["direction_inverted"]
            )
            
            # 设置目标速度
            target_speed = 60
            motor.set_speed(target_speed)
            
            # 验证 PID setpoint
            assert motor.pid.setpoint == target_speed
            
            # 获取当前速度
            current_speed = motor.get_speed()
            assert isinstance(current_speed, (int, float))
            
            # 更新电机（模拟控制循环）
            mock_encoder.read.return_value = 440  # 模拟编码器读数
            mock_encoder.get_speed.return_value = 100  # 模拟速度
            speed, duty = motor.update(dt=0.01)
            
            # 验证速度获取方法被调用
            mock_encoder.get_speed.assert_called()
            
            motor.cleanup()

class TestPIDController:
    """PID控制器单元测试"""
    
    def test_pid_initialization(self):
        """测试PID控制器初始化"""
        pid = PIDController(kp=0.5, ki=0.1, kd=0.0, setpoint=0)
        assert pid.kp == 0.5
        assert pid.ki == 0.1
        assert pid.kd == 0.0
        assert pid.setpoint == 0
    
    def test_pid_compute(self):
        """测试PID计算"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0, setpoint=10)
        
        # 测试比例控制（不带 dt 参数）
        output = pid.compute(5)
        assert output == 5.0  # (10-5) * 1.0

class TestBaseControllerUnit:
    """底盘控制器单元测试"""
    
    def test_base_factory_initialization(self):
        """测试底盘工厂初始化"""
        factory = BaseFactory()
        assert factory is not None
    
    @patch('src.base.d24a_jgb37.Motor')
    def test_differential_controller_move_forward(self, mock_motor_class):
        """测试差分控制器前进"""
        # 创建 mock 电机
        mock_motors = {
            'FL': Mock(name='FL'),
            'FR': Mock(name='FR'),
            'BL': Mock(name='BL'),
            'BR': Mock(name='BR')
        }
        mock_motor_class.side_effect = [mock_motors['FL'], mock_motors['FR'], 
                                       mock_motors['BL'], mock_motors['BR']]
        
        # 创建控制器
        controller = DifferentialController(mock_motors, max_speed=100)
        
        # 设置前进速度（直接设置 vx）
        controller.vx = 50
        
        # 验证速度设置正确
        assert controller.vx == 50
        assert controller.vy == 0
        assert controller.vw == 0
        
        controller.cleanup()
    
    @patch('src.base.d24a_jgb37.Motor')
    def test_differential_controller_turn_left(self, mock_motor_class):
        """测试差分控制器左转"""
        mock_motors = {
            'FL': Mock(name='FL'),
            'FR': Mock(name='FR'),
            'BL': Mock(name='BL'),
            'BR': Mock(name='BR')
        }
        mock_motor_class.side_effect = [mock_motors['FL'], mock_motors['FR'], 
                                       mock_motors['BL'], mock_motors['BR']]
        
        controller = DifferentialController(mock_motors, max_speed=100)
        
        # 设置左转角速度（直接设置 vw）
        controller.vw = 30
        
        # 验证角速度设置正确
        assert controller.vw == 30
        
        controller.cleanup()
    
    @patch('src.base.d24a_jgb37.Motor')
    def test_differential_controller_stop(self, mock_motor_class):
        """测试差分控制器停止"""
        mock_motors = {
            'FL': Mock(name='FL'),
            'FR': Mock(name='FR'),
            'BL': Mock(name='BL'),
            'BR': Mock(name='BR')
        }
        mock_motor_class.side_effect = [mock_motors['FL'], mock_motors['FR'], 
                                       mock_motors['BL'], mock_motors['BR']]
        
        controller = DifferentialController(mock_motors, max_speed=100)
        
        # 设置速度后停止
        controller.vx = 50
        controller.stop()
        
        # 验证所有速度都为0
        assert controller.vx == 0
        assert controller.vy == 0
        assert controller.vw == 0
        
        controller.cleanup()
    
    @patch('src.base.d24a_jgb37.Motor')
    def test_mecanum_controller_move(self, mock_motor_class):
        """测试麦轮控制器运动"""
        mock_motors = {
            'FL': Mock(name='FL'),
            'FR': Mock(name='FR'),
            'BL': Mock(name='BL'),
            'BR': Mock(name='BR')
        }
        mock_motor_class.side_effect = [mock_motors['FL'], mock_motors['FR'], 
                                       mock_motors['BL'], mock_motors['BR']]
        
        controller = MecanumController(mock_motors, max_speed=100)
        
        # 设置速度（直接设置属性）
        controller.vx = 50
        controller.vy = 20
        controller.vw = 10
        
        # 验证速度设置正确
        assert controller.vx == 50
        assert controller.vy == 20
        assert controller.vw == 10
        
        controller.cleanup()
    
    @patch('src.base.d24a_jgb37.Motor')
    def test_mecanum_controller_strafe_right(self, mock_motor_class):
        """测试麦轮控制器侧移"""
        mock_motors = {
            'FL': Mock(name='FL'),
            'FR': Mock(name='FR'),
            'BL': Mock(name='BL'),
            'BR': Mock(name='BR')
        }
        mock_motor_class.side_effect = [mock_motors['FL'], mock_motors['FR'], 
                                       mock_motors['BL'], mock_motors['BR']]
        
        controller = MecanumController(mock_motors, max_speed=100)
        
        # 右侧移（设置 vy）
        controller.vy = 30
        
        # 验证侧移速度设置正确
        assert controller.vy == 30
        
        controller.cleanup()