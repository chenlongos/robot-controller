# src/base/__init__.py
"""底盘模块 - 注册所有底盘类型"""

from src.abstract.base_factory import BaseFactory
from .differential_base import DifferentialBase
from .mecanum_base import MecanumBase

BaseFactory.register_base("differential", DifferentialBase)
BaseFactory.register_base("mecanum", MecanumBase)