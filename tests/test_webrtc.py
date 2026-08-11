# tests/test_webrtc.py
"""WebRTC 视频推流测试程序

启动 WebRTC 服务器，将摄像头画面实时推流到远程浏览器。
运行后，在远程电脑浏览器中访问 http://<机器人IP>:8080 即可查看视频流。
"""

import os
import sys
import time
import socket
import logging

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config_loader import load_config
from src.abstract.camera_factory import CameraFactory
import src.camera  # 注册 USB 摄像头
from src.web.webrtc_server import start_webrtc_server, push_frame, is_available
from _robot_select import select_robot


def get_local_ip() -> str:
    """获取本机局域网 IP 地址，用于生成远程访问 URL。"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def main():
    # 选择机器人
    robot_name = select_robot()
    if robot_name is None:
        return

    config = load_config(robot_name)

    # 检查 WebRTC 依赖
    if not is_available():
        print("错误: WebRTC 依赖未安装（aiortc/aiohttp），请先安装依赖")
        sys.exit(1)

    # 初始化摄像头
    camera_config = config.device.hardware.camera
    camera_type = camera_config.type
    camera = CameraFactory.create_camera(camera_type, camera_config)
    if not camera.is_opened():
        print("错误: 摄像头打开失败")
        sys.exit(1)
    print(f"摄像头初始化成功: 类型={camera_type}, 分辨率={camera_config.resolution}")

    # 启动 WebRTC 服务器
    port = 8080
    start_webrtc_server(port=port)

    local_ip = get_local_ip()
    print("\n" + "=" * 50)
    print("  WebRTC 视频推流已启动")
    print(f"  远程访问地址: http://{local_ip}:{port}")
    print("  在远程电脑浏览器中打开上述地址即可查看视频流")
    print("  按 Ctrl+C 停止推流")
    print("=" * 50 + "\n")

    # 推流循环
    frame_interval = 1.0 / 15.0  # 15fps
    last_push_time = 0.0

    try:
        while True:
            current_time = time.time()
            if current_time - last_push_time >= frame_interval:
                frame = camera.capture(flush_frames=1)
                if frame is not None:
                    push_frame(frame)
                else:
                    logging.warning("无法捕获帧，跳过推流")
                last_push_time = current_time
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n收到中断信号，正在退出...")

    finally:
        camera.release()
        print("摄像头资源已释放，程序退出")


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s"
    )
    main()
