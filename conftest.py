"""pytest 配置文件 - 实现动态日志文件名"""
import os
import time
import pytest

def pytest_configure(config):
    """配置 pytest，设置动态日志文件名"""
    # 创建 logs 目录（如果不存在）
    logs_dir = "logs"
    os.makedirs(logs_dir, exist_ok=True)
    
    # 生成带时间戳的日志文件名
    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    log_file_path = os.path.join(logs_dir, f"pytest_{timestamp}.log")
    
    # 设置日志文件路径
    config.option.log_file = log_file_path
    
    print(f"日志文件将输出到: {log_file_path}")