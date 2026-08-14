# src/utils/image_saver.py
"""视频帧保存工具"""

import os
import time
import logging
import threading
import cv2

from src.config_loader import load_config
from src.camera.usb_camera import USBCamera
from src.web.webrtc_server import start_webrtc_server, push_frame, is_available as webrtc_available

# 项目根目录: src/utils/ -> src/ -> 项目根
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def save_picture(frame_img, vision_module=None, tennis_dets=None, bucket_dets=None,
                 state=None, observation=None, label=""):
    """保存视频帧到 tests/pic 目录

    可保存以下任一类型的图片：
    - 带网球检测框的图片（传入 tennis_dets）
    - 带桶检测框的图片（传入 bucket_dets）
    - 纯图片（不传 vision_module，或不传任何检测框）

    Args:
        frame_img: 待保存的图像帧
        vision_module: 视觉模块对象，用于绘制检测框（可选）
        tennis_dets: 网球检测结果列表（可选）
        bucket_dets: 桶检测结果列表（可选）
        state: 状态机状态（可选）
        observation: 观测信息字典（可选）
        label: 文件名中追加的自定义字符串（可选）

    Returns:
        str: 保存的文件路径
    """
    pic_dir = os.path.join(_PROJECT_ROOT, 'tests', 'pic')
    os.makedirs(pic_dir, exist_ok=True)

    # 仅在提供 vision_module 且存在任一检测结果时才绘制检测框，否则保存纯图片
    has_tennis = bool(tennis_dets)
    has_bucket = bool(bucket_dets)
    if vision_module is not None and (has_tennis or has_bucket):
        display = vision_module.draw_tracking_info(
            frame_img, tennis_dets or [], bucket_dets or [], state, observation or {})
    else:
        display = frame_img

    # 构造文件名
    parts = []
    if label:
        parts.append(label)
    parts.append(time.strftime('%Y%m%d_%H%M%S'))
    fname = "_".join(parts) + ".jpg"
    fpath = os.path.join(pic_dir, fname)
    cv2.imwrite(fpath, display)
    logging.info(f"已保存帧到 {fpath}")
    return fpath


def main():
    """交互式保存图片循环（带 WebRTC 连续推流）

    在后台线程中持续从 USB 摄像头取帧并推流到 WebRTC，保证推流不被打断；
    主线程处理用户交互，保存图片时复用后台线程采集到的最新帧。
    - 选择保存时可输入一个字符串追加到文件名
    - 选择不保存则退出程序
    """
    config = load_config()
    camera = USBCamera({
        'device_id': config.device.hardware.camera.device_id,
        'resolution': config.device.hardware.camera.resolution,
        'frame_rate': config.device.hardware.camera.frame_rate,
    })

    if not camera.is_opened():
        print("摄像头未打开，退出程序")
        return

    start_webrtc_server(port=8080)

    print("=" * 50)
    print("图片保存工具（WebRTC 推流中）")
    print("=" * 50)
    print("推流地址: http://<本机IP>:8080")
    print("输入 'y' 保存图片，输入其他任意键退出")

    # 后台推流线程与主线程共享的最新帧
    latest_frame = {'frame': None}
    frame_lock = threading.Lock()
    streaming_flag = {'running': True}

    def streaming_loop():
        """连续推流：不断取帧并推送到 WebRTC，同时更新共享的最新帧"""
        while streaming_flag['running']:
            frame = camera.capture()
            if frame is None:
                time.sleep(0.01)
                continue
            with frame_lock:
                latest_frame['frame'] = frame
            if webrtc_available():
                push_frame(frame)

    stream_thread = threading.Thread(target=streaming_loop, daemon=True)
    stream_thread.start()

    try:
        while True:
            choice = input("\n是否保存图片？(y/n): ").strip().lower()
            if choice != 'y':
                print("退出程序")
                break

            with frame_lock:
                frame = latest_frame['frame']
            if frame is None:
                print("尚未采集到帧，跳过本次保存")
                continue

            label = input("请输入要加入文件名的字符串（可直接回车跳过）: ").strip()
            fpath = save_picture(frame, label=label)
            print(f"已保存: {fpath}")
    finally:
        streaming_flag['running'] = False
        camera.release()


if __name__ == "__main__":
    main()
